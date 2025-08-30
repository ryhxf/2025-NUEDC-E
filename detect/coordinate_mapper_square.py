"""
方块跟踪坐标映射模块 - 图像坐标到舵机角度增量的数学模型
专为串行总线舵机设计，支持相对位置控制
"""
import json
import numpy as np
import os
import warnings
import time
import logging
warnings.filterwarnings('ignore')

# **修复：添加logger配置**
logger = logging.getLogger("SquareCoordinateMapper")

# 安全导入依赖库，提供回退方案
try:
    from scipy.interpolate import RBFInterpolator
    SCIPY_AVAILABLE = True
    print("✓ [坐标映射] scipy库可用")
except ImportError:
    print("⚠️ [坐标映射] scipy库不可用，将使用简化插值")
    SCIPY_AVAILABLE = False

try:
    from sklearn.linear_model import LinearRegression
    from sklearn.preprocessing import PolynomialFeatures
    from sklearn.pipeline import Pipeline
    SKLEARN_AVAILABLE = True
    print("✓ [坐标映射] sklearn库可用")
except ImportError:
    print("⚠️ [坐标映射] sklearn库不可用，将使用基础线性插值")
    SKLEARN_AVAILABLE = False

class SimpleLinearInterpolator:
    """简单线性插值器 - 当scipy/sklearn不可用时的回退方案"""
    
    def __init__(self, points, values):
        self.points = np.array(points)
        self.values = np.array(values)
        
        # 计算线性回归系数
        X = self.points
        y = self.values
        
        # 使用最小二乘法计算参数：y = ax + by + c
        A = np.column_stack([X[:, 0], X[:, 1], np.ones(len(X))])
        
        try:
            ATA_inv = np.linalg.inv(A.T @ A)
            self.coeffs = ATA_inv @ A.T @ y
            print(f"✓ [简单插值] 线性模型参数: a={self.coeffs[0]:.4f}, b={self.coeffs[1]:.4f}, c={self.coeffs[2]:.4f}")
        except np.linalg.LinAlgError:
            self.coeffs = np.linalg.pinv(A) @ y
            print(f"⚠️ [简单插值] 使用伪逆解决奇异矩阵")
    
    def __call__(self, query_points):
        """预测新点的值"""
        query_points = np.array(query_points)
        if query_points.ndim == 1:
            query_points = query_points.reshape(1, -1)
        
        results = []
        for point in query_points:
            x, y = point[0], point[1]
            value = self.coeffs[0] * x + self.coeffs[1] * y + self.coeffs[2]
            results.append(value)
        
        return np.array(results)
    
    def predict(self, query_points):
        """sklearn兼容的预测接口"""
        return self.__call__(query_points)

class SquareCoordinateMapper:
    """方块跟踪坐标映射器 - 基于YOLO+OpenCV检测的四边形中心点偏差的角度增量转换"""
    
    def __init__(self, calibration_file="/root/square/detect/square_servo_calibration.json"):
        self.calibration_file = calibration_file
        self.calibration_data = None
        self.image_width = 640
        self.image_height = 480
        
        # **串行总线舵机参数**
        self.servo_resolution = 4096
        self.angle_per_unit = 360.0 / self.servo_resolution
        self.position_min = -32767
        self.position_max = 32768
        
        # **设备参数**
        self.servo_port = '/dev/ttyACM0'
        self.servo_baudrate = 1000000
        self.x_servo_id = 1
        self.y_servo_id = 2
        
        # **映射范围限制（增量角度）**
        self.max_delta_angle_x = 15.0
        self.max_delta_angle_y = 15.0
        
        # **重要：图像中心点作为映射基准（目标位置）**
        self.image_center_x = 0  # 将在加载标定时更新
        self.image_center_y = 0
        
        # **新增：目标中心点（可配置偏移）**
        self.target_center_x = 0  # 目标激光点位置
        self.target_center_y = 0
        
        # **新增：极性控制参数**
        self.x_polarity = -1  # X轴极性：1=正常，-1=反向
        self.y_polarity = -1  # Y轴极性：1=正常，-1=反向
        
        # 插值器
        self.x_interpolator = None
        self.y_interpolator = None
        
        # 根据可用库选择映射方法
        if SCIPY_AVAILABLE:
            self.mapping_method = "rbf"
        elif SKLEARN_AVAILABLE:
            self.mapping_method = "linear"
        else:
            self.mapping_method = "simple"
        
        print(f"✓ [坐标映射] 选择映射方法: {self.mapping_method}")
        print(f"✓ [坐标映射] 舵机分辨率: {self.servo_resolution} (每度约{self.servo_resolution/360:.1f}单位)")
        
        # 统计信息
        self.total_conversions = 0
        self.successful_conversions = 0
        
        # 加载标定数据
        self.load_calibration()
    
    def load_calibration(self):
        """加载标定数据并建立映射模型"""
        try:
            if not os.path.exists(self.calibration_file):
                print(f"⚠️ [坐标映射] 标定文件不存在: {self.calibration_file}")
                print(f"📝 [坐标映射] 请先运行 coordinate_calibration_square.py 进行坐标标定")
                return False
            
            print(f"📁 [坐标映射] 加载标定文件: {self.calibration_file}")
            
            with open(self.calibration_file, 'r') as f:
                self.calibration_data = json.load(f)
            
            # **新增：读取设备连接参数**
            servo_config = self.calibration_data.get('servo_config', {})
            if servo_config:
                self.x_servo_id = servo_config.get('x_servo_id', 1)
                self.y_servo_id = servo_config.get('y_servo_id', 2)
                self.servo_resolution = servo_config.get('resolution', 4096)
                self.angle_per_unit = servo_config.get('angle_per_unit', 360.0 / 4096)
                
                print(f"📡 [坐标映射] 舵机配置: X轴ID={self.x_servo_id}, Y轴ID={self.y_servo_id}")
                print(f"⚙️ [坐标映射] 分辨率: {self.servo_resolution} 单位/360°")
            
            # **新增：显示标定时的连接信息**
            center_position = self.calibration_data.get('center_position', {})
            if center_position:
                center_x = center_position.get('x', 0)
                center_y = center_position.get('y', 0)
                center_x_angle = center_x * self.angle_per_unit
                center_y_angle = center_y * self.angle_per_unit
                print(f"📌 [坐标映射] 标定中心: X={center_x}({center_x_angle:.1f}°), Y={center_y}({center_y_angle:.1f}°)")
            
            # **新增：读取极性配置**
            polarity_config = self.calibration_data.get('polarity_config', {})
            if polarity_config:
                self.x_polarity = polarity_config.get('x_polarity', -1)
                self.y_polarity = polarity_config.get('y_polarity', -1)
                print(f"🔄 [坐标映射] 极性配置: X轴={self.x_polarity}, Y轴={self.y_polarity}")
            else:
                print(f"🔄 [坐标映射] 使用默认极性: X轴={self.x_polarity}, Y轴={self.y_polarity}")
            
            # 提取图像尺寸
            image_size = self.calibration_data.get('image_size', {})
            self.image_width = image_size.get('width', 640)
            self.image_height = image_size.get('height', 480)
            
            # 提取增量限制
            delta_limits = self.calibration_data.get('delta_limits', {})
            self.max_delta_angle_x = delta_limits.get('max_delta_x', 15.0)
            self.max_delta_angle_y = delta_limits.get('max_delta_y', 15.0)
            
            # 提取标定点
            points = self.calibration_data.get('calibration_points', [])
            
            if len(points) < 4:
                print(f"⚠️ [坐标映射] 标定点不足: {len(points)} < 4")
                return False
            
            print(f"✓ [坐标映射] 加载标定数据: {len(points)} 个标定点")
            print(f"✓ [坐标映射] 图像尺寸: {self.image_width}x{self.image_height}")
            print(f"✓ [坐标映射] 增量范围: X(±{self.max_delta_angle_x}°), Y(±{self.max_delta_angle_y}°)")
            
            # 建立映射模型
            return self._build_mapping_model(points)
            
        except Exception as e:
            print(f"❌ [坐标映射] 加载标定数据失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _build_mapping_model(self, points):
        """**修改：建立基于YOLO+OpenCV检测中心点偏差的坐标映射模型**"""
        try:
            # **关键修改：计算图像中心点和目标中心点**
            self.image_center_x = self.image_width / 2
            self.image_center_y = self.image_height / 2
            
            # **目标中心点可以有偏移（从配置读取）**
            target_config = self.calibration_data.get('target_config', {})
            offset_x = target_config.get('offset_x', 0)
            offset_y = target_config.get('offset_y', 0)
            
            self.target_center_x = self.image_center_x + offset_x
            self.target_center_y = self.image_center_y + offset_y
            
            print(f"📌 [坐标映射] 图像中心: ({self.image_center_x}, {self.image_center_y})")
            print(f"🎯 [坐标映射] 目标中心: ({self.target_center_x}, {self.target_center_y}) (偏移: {offset_x:+d}, {offset_y:+d})")
            
            # **重要：计算标定点的偏差值**
            # 标定时：人工点击位置代表"激光应该照射的位置"
            # 运行时：YOLO+OpenCV检测的中心代表"四边形实际中心位置"
            # 偏差 = 检测中心 - 目标中心
            offset_coords = []  # 存储相对于目标中心的偏差值
            delta_x_angles = []  # 角度增量
            delta_y_angles = []  # 角度增量
            
            for point in points:
                # **标定时的逻辑：点击位置相对于目标中心的偏差**
                # 这里的逻辑是：人工点击位置 - 目标中心 = 应该补偿的偏差
                offset_x = point['image_x'] - self.target_center_x
                offset_y = point['image_y'] - self.target_center_y
                
                offset_coords.append([offset_x, offset_y])
                delta_x_angles.append(point['delta_x'])
                delta_y_angles.append(point['delta_y'])
            
            offset_coords = np.array(offset_coords)
            delta_x_angles = np.array(delta_x_angles)
            delta_y_angles = np.array(delta_y_angles)
            
            print(f"🔧 [坐标映射] 建立YOLO+OpenCV中心点偏差映射模型，使用方法: {self.mapping_method}")
            print(f"📊 [坐标映射] 偏差值数据范围:")
            print(f"   偏差X: {offset_coords[:, 0].min():.1f} - {offset_coords[:, 0].max():.1f} 像素")
            print(f"   偏差Y: {offset_coords[:, 1].min():.1f} - {offset_coords[:, 1].max():.1f} 像素")
            print(f"   增量X: {delta_x_angles.min():.2f}° - {delta_x_angles.max():.2f}°")
            print(f"   增量Y: {delta_y_angles.min():.2f}° - {delta_y_angles.max():.2f}°")
            
            if self.mapping_method == "rbf" and SCIPY_AVAILABLE:
                # 径向基函数插值
                print("🎯 [坐标映射] 使用RBF插值（基于偏差值）")
                self.x_interpolator = RBFInterpolator(
                    offset_coords, delta_x_angles, 
                    kernel='thin_plate_spline',
                    smoothing=0.1
                )
                self.y_interpolator = RBFInterpolator(
                    offset_coords, delta_y_angles, 
                    kernel='thin_plate_spline',
                    smoothing=0.1
                )
                
            elif self.mapping_method == "linear" and SKLEARN_AVAILABLE:
                # 线性回归
                print("📈 [坐标映射] 使用sklearn线性回归（基于偏差值）")
                self.x_interpolator = LinearRegression()
                self.y_interpolator = LinearRegression()
                
                self.x_interpolator.fit(offset_coords, delta_x_angles)
                self.y_interpolator.fit(offset_coords, delta_y_angles)
                
            else:
                # 简单线性插值（回退方案）
                print("🔧 [坐标映射] 使用简单线性插值（基于偏差值，回退方案）")
                self.mapping_method = "simple"
                self.x_interpolator = SimpleLinearInterpolator(offset_coords, delta_x_angles)
                self.y_interpolator = SimpleLinearInterpolator(offset_coords, delta_y_angles)
            
            print(f"✅ [坐标映射] YOLO+OpenCV中心点偏差映射模型建立成功")
            
            # 测试模型
            test_success = self._test_yolo_opencv_model()
            if test_success:
                print(f"✅ [坐标映射] YOLO+OpenCV模型测试通过")
                return True
            else:
                print(f"❌ [坐标映射] YOLO+OpenCV模型测试失败")
                return False
            
        except Exception as e:
            print(f"❌ [坐标映射] 建立YOLO+OpenCV偏差映射模型失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _test_yolo_opencv_model(self):
        """**新增：测试YOLO+OpenCV模型**"""
        try:
            # 测试零偏差（检测中心与目标中心重合）
            test_offset_x, test_offset_y = 0.0, 0.0
            delta_x, delta_y = self.detection_center_to_delta_angle(
                self.target_center_x, self.target_center_y
            )
            
            if delta_x is not None and delta_y is not None:
                print(f"✅ [坐标映射] 零偏差测试: 检测中心=目标中心 -> 增量({delta_x:.2f}°,{delta_y:.2f}°)")
                
                # 测试一些典型的检测偏差
                test_cases = [
                    (self.target_center_x + 50, self.target_center_y, "右偏移50像素"),
                    (self.target_center_x - 50, self.target_center_y, "左偏移50像素"),
                    (self.target_center_x, self.target_center_y + 50, "下偏移50像素"),
                    (self.target_center_x, self.target_center_y - 50, "上偏移50像素"),
                    (self.target_center_x + 30, self.target_center_y + 30, "右下偏移"),
                    (self.target_center_x - 30, self.target_center_y - 30, "左上偏移")
                ]
                
                print("🔍 [坐标映射] 典型检测偏差测试:")
                for detected_x, detected_y, description in test_cases:
                    delta_x, delta_y = self.detection_center_to_delta_angle(detected_x, detected_y)
                    if delta_x is not None:
                        offset_x = detected_x - self.target_center_x
                        offset_y = detected_y - self.target_center_y
                        print(f"   {description}: 偏差({offset_x:+.0f},{offset_y:+.0f}) -> 增量({delta_x:+.2f}°,{delta_y:+.2f}°)")
                
                return True
            else:
                print(f"❌ [坐标映射] 零偏差测试失败")
                return False
                
        except Exception as e:
            print(f"❌ [坐标映射] YOLO+OpenCV模型测试异常: {e}")
            return False
    
    def detection_center_to_delta_angle(self, detected_center_x, detected_center_y):
        """
        **主要接口：将YOLO+OpenCV检测到的四边形中心点转换为舵机角度增量**
        
        Args:
            detected_center_x: YOLO+OpenCV检测到的四边形中心X坐标
            detected_center_y: YOLO+OpenCV检测到的四边形中心Y坐标
            
        Returns:
            tuple: (delta_x_angle, delta_y_angle) 或 (None, None) 如果失败
        """
        try:
            # **关键：计算检测中心相对于目标中心的偏差**
            # 偏差 = 检测位置 - 目标位置
            # 正偏差表示检测位置在目标位置右侧/下方
            offset_x = detected_center_x - self.target_center_x
            offset_y = detected_center_y - self.target_center_y
            
            # 使用偏差值进行映射
            delta_x, delta_y = self.offset_to_delta_angle(offset_x, offset_y)
            
            if delta_x is not None and delta_y is not None:
                # **修复：使用print替代logger.debug，避免logger未定义错误**
                print(f"🎯 [坐标映射] YOLO+OpenCV中心映射: 检测({detected_center_x:.1f},{detected_center_y:.1f}) - 目标({self.target_center_x:.1f},{self.target_center_y:.1f}) = 偏差({offset_x:+.1f},{offset_y:+.1f}) -> 增量({delta_x:+.2f}°,{delta_y:+.2f}°)")
            
            return delta_x, delta_y
            
        except Exception as e:
            print(f"❌ [坐标映射] YOLO+OpenCV中心映射失败: {e}")
            return None, None

    def get_statistics(self):
        """获取转换统计信息"""
        success_rate = 0.0
        if self.total_conversions > 0:
            success_rate = self.successful_conversions / self.total_conversions * 100
        
        return {
            'total_conversions': self.total_conversions,
            'successful_conversions': self.successful_conversions,
            'success_rate': success_rate,
            'mapping_method': self.mapping_method,
            'calibration_points': len(self.calibration_data.get('calibration_points', [])) if self.calibration_data else 0,
            'model_ready': self.x_interpolator is not None and self.y_interpolator is not None,
            'servo_resolution': self.servo_resolution,
            'angle_per_unit': self.angle_per_unit,
            'max_delta_x': self.max_delta_angle_x,
            'max_delta_y': self.max_delta_angle_y
        }

    def delta_angle_to_position(self, delta_x_angle, delta_y_angle):
        """
        **新增：将角度增量转换为舵机位置增量**
        
        Args:
            delta_x_angle: X轴角度增量（度）
            delta_y_angle: Y轴角度增量（度）
            
        Returns:
            tuple: (delta_x_position, delta_y_position) 舵机位置增量
        """
        try:
            # 角度增量转换为位置增量
            delta_x_position = int(delta_x_angle / self.angle_per_unit)
            delta_y_position = int(delta_y_angle / self.angle_per_unit)
            
            # 限制位置增量范围（防止超出舵机限制）
            max_position_delta = int(self.max_delta_angle_x / self.angle_per_unit)
            delta_x_position = max(-max_position_delta, min(max_position_delta, delta_x_position))
            
            max_position_delta = int(self.max_delta_angle_y / self.angle_per_unit)
            delta_y_position = max(-max_position_delta, min(max_position_delta, delta_y_position))
            
            return delta_x_position, delta_y_position
            
        except Exception as e:
            print(f"❌ [坐标映射] 角度增量转换失败: {e}")
            return 0, 0

    def image_to_position_delta(self, image_x, image_y):
        """
        **新增：直接将图像坐标转换为舵机位置增量**
        
        Args:
            image_x: 图像X坐标 (像素)
            image_y: 图像Y坐标 (像素)
            
        Returns:
            tuple: (delta_x_position, delta_y_position) 或 (None, None) 如果失败
        """
        # 先转换为角度增量
        delta_x_angle, delta_y_angle = self.image_to_delta_angle(image_x, image_y)
        
        if delta_x_angle is None or delta_y_angle is None:
            return None, None
        
        # 再转换为位置增量
        return self.delta_angle_to_position(delta_x_angle, delta_y_angle)

    def offset_to_delta_angle(self, offset_x, offset_y):
        """
        **新增：将像素偏差转换为舵机角度增量**
        
        Args:
            offset_x: X轴像素偏差（可正可负）
            offset_y: Y轴像素偏差（可正可负）
            
        Returns:
            tuple: (delta_x_angle, delta_y_angle) 或 (None, None) 如果失败
        """
        self.total_conversions += 1
        
        try:
            if self.x_interpolator is None or self.y_interpolator is None:
                print(f"❌ [坐标映射] 偏差值映射模型未初始化")
                return None, None
            
            # **限制偏差值范围，避免超出标定范围**
            max_offset = min(self.image_width, self.image_height) / 2
            offset_x = max(-max_offset, min(max_offset, offset_x))
            offset_y = max(-max_offset, min(max_offset, offset_y))
            
            # 准备输入数据
            query_point = np.array([[offset_x, offset_y]])
            
            # 预测舵机角度增量
            if self.mapping_method == "rbf" and SCIPY_AVAILABLE:
                delta_x = float(self.x_interpolator(query_point)[0])
                delta_y = float(self.y_interpolator(query_point)[0])
            else:  # linear, simple
                delta_x = float(self.x_interpolator.predict(query_point)[0])
                delta_y = float(self.y_interpolator.predict(query_point)[0])
            
            # **新增：应用极性控制**
            delta_x *= self.x_polarity
            delta_y *= self.y_polarity
            
            # 限制增量范围
            delta_x = max(-self.max_delta_angle_x, min(self.max_delta_angle_x, delta_x))
            delta_y = max(-self.max_delta_angle_y, min(self.max_delta_angle_y, delta_y))
            
            self.successful_conversions += 1
            
            return delta_x, delta_y
            
        except Exception as e:
            print(f"❌ [坐标映射] 偏差值转换失败: {e}")
            return None, None
    
    def set_polarity(self, x_polarity, y_polarity):
        """**新增：设置极性控制**
        
        Args:
            x_polarity: X轴极性 (1=正常, -1=反向)
            y_polarity: Y轴极性 (1=正常, -1=反向)
        """
        self.x_polarity = x_polarity
        self.y_polarity = y_polarity
        print(f"🔄 [坐标映射] 极性已更新: X轴={self.x_polarity}, Y轴={self.y_polarity}")
    
    def get_polarity_info(self):
        """**新增：获取极性信息**"""
        return {
            'x_polarity': self.x_polarity,
            'y_polarity': self.y_polarity,
            'x_description': '正常' if self.x_polarity == 1 else '反向',
            'y_description': '正常' if self.y_polarity == 1 else '反向'
        }
    
    def get_target_info(self):
        """获取目标信息"""
        return {
            'image_center': [self.image_center_x, self.image_center_y],
            'target_center': [self.target_center_x, self.target_center_y],
            'offset': [self.target_center_x - self.image_center_x, self.target_center_y - self.image_center_y]
        }

    def image_to_delta_angle(self, image_x, image_y):
        """兼容接口：将图像坐标转换为角度增量"""
        return self.detection_center_to_delta_angle(image_x, image_y)

# 测试代码
if __name__ == "__main__":
    print("🧪 [坐标映射] 测试方块跟踪坐标映射模块")
    
    mapper = SquareCoordinateMapper()
    
    if mapper.calibration_data:
        # 验证模型
        mapper.validate_model()
        
        # 测试一些坐标点
        test_points = [
            (320, 240),  # 图像中心
            (160, 120),  # 左上
            (480, 360),  # 右下
            (100, 300),  # 左下
            (540, 180),  # 右上
        ]
        
        print("\n🎯 [坐标映射] 测试坐标转换:")
        for x, y in test_points:
            delta_x, delta_y = mapper.image_to_delta_angle(x, y)
            if delta_x is not None:
                pos_delta_x, pos_delta_y = mapper.delta_angle_to_position(delta_x, delta_y)
                print(f"  图像({x}, {y}) -> 增量角度(Δ{delta_x:.2f}°, Δ{delta_y:.2f}°) -> 位置增量({pos_delta_x:+d}, {pos_delta_y:+d})")
            else:
                print(f"  图像({x}, {y}) -> 转换失败")
        
        # 显示统计信息
        stats = mapper.get_statistics()
        print(f"\n📊 [坐标映射] 统计信息: {stats}")
    
    else:
        print("📝 [坐标映射] 请先运行标定程序生成标定数据")
