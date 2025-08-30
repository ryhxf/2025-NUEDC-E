"""
方块跟踪系统UI模块 - 专门处理图像显示和用户界面
分离显示逻辑，减少主控制文件的代码量
"""
import cv2
import time
import logging
import numpy as np

logger = logging.getLogger("SquareTrackerUI")

class SquareTrackerUI:
    """方块跟踪系统UI界面类"""
    
    def __init__(self, config: dict):
        self.config = config
        self.enabled = config.get('enable_display', False)
        
        # 窗口配置
        self.window_name = config.get('display_window_name', 'Square Tracker - YOLO+OpenCV Real-time')
        
        # 显示选项
        self.display_fps_counter = config.get('display_fps_counter', True)
        self.display_detection_info = config.get('display_detection_info', True)
        self.display_servo_status = config.get('display_servo_status', True)
        self.display_mapping_stats = config.get('display_mapping_stats', True)
        
        # FPS统计
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # 状态
        self.window_created = False
        
        if self.enabled:
            self._create_window()
    
    def _create_window(self):
        """创建显示窗口"""
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            self.window_created = True
            logger.info(f"✓ 图像显示窗口已创建: {self.window_name}")
        except Exception as e:
            logger.error(f"创建显示窗口失败: {e}")
            self.enabled = False
    
    def update_fps(self):
        """更新FPS计算"""
        self.fps_counter += 1
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def add_fps_overlay(self, frame):
        """添加FPS覆盖层"""
        if not self.display_fps_counter:
            return frame
        
        try:
            # FPS显示
            fps_text = f"FPS: {self.current_fps:.1f}"
            cv2.putText(frame, fps_text, (frame.shape[1] - 120, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 处理帧数显示
            frame_text = f"Frame: {getattr(self, 'frame_count', 0)}"
            cv2.putText(frame, frame_text, (frame.shape[1] - 150, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            return frame
        except Exception as e:
            logger.error(f"添加FPS覆盖层失败: {e}")
            return frame
    
    def draw_ui_elements(self, frame, tracker_state: dict):
        """绘制UI元素"""
        if not self.enabled:
            return frame
        
        try:
            # 获取目标中心点信息
            target_info = tracker_state.get('target_info', {})
            target_center = target_info.get('target_center', [320, 240])
            image_center = target_info.get('image_center', [320, 240])
            offset = target_info.get('offset', [0, 0])
            
            target_center_x, target_center_y = target_center
            image_center_x, image_center_y = image_center
            offset_x, offset_y = offset
            
            # 绘制图像处理区域边界
            if self.display_detection_info:
                border_margin = 50
                cv2.rectangle(frame, 
                             (border_margin, border_margin), 
                             (frame.shape[1] - border_margin, frame.shape[0] - border_margin), 
                             (100, 100, 100), 1)
            
            # 绘制图像中心（蓝色圆圈）
            cv2.circle(frame, (int(image_center_x), int(image_center_y)), 8, (255, 0, 0), 2)
            cv2.putText(frame, "IMG CENTER", (int(image_center_x) - 35, int(image_center_y) - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            # 绘制目标中心（红色十字）
            cv2.circle(frame, (int(target_center_x), int(target_center_y)), 12, (0, 0, 255), 2)
            cv2.line(frame, (int(target_center_x)-20, int(target_center_y)), 
                     (int(target_center_x)+20, int(target_center_y)), (0, 0, 255), 2)
            cv2.line(frame, (int(target_center_x), int(target_center_y)-20), 
                     (int(target_center_x), int(target_center_y)+20), (0, 0, 255), 2)
            cv2.putText(frame, "LASER TARGET", (int(target_center_x) - 35, int(target_center_y) - 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
            # 绘制偏移连接线
            if offset_x != 0 or offset_y != 0:
                cv2.line(frame, (int(image_center_x), int(image_center_y)), 
                         (int(target_center_x), int(target_center_y)), (255, 255, 255), 2)
            
            # 绘制状态信息
            self._draw_status_info(frame, tracker_state)
            
            return frame
            
        except Exception as e:
            logger.error(f"绘制UI元素失败: {e}")
            return frame
    
    def _draw_status_info(self, frame, tracker_state: dict):
        """绘制状态信息"""
        y_offset = 30
        line_height = 25
        
        # 跟踪状态
        is_tracking = tracker_state.get('is_tracking', False)
        status_color = (0, 255, 0) if is_tracking else (0, 255, 255)
        status_text = f"Tracking: {'ON' if is_tracking else 'OFF'}"
        cv2.putText(frame, status_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        y_offset += line_height
        
        # 目标检测状态
        target_found = tracker_state.get('target_found', False)
        target_color = (0, 255, 0) if target_found else (0, 0, 255)
        target_text = f"Target: {'FOUND' if target_found else 'LOST'}"
        cv2.putText(frame, target_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, target_color, 2)
        y_offset += line_height
        
        # 检测方法显示
        detection_method = tracker_state.get('detection_method', 'Unknown')
        if detection_method != 'Unknown':
            method_text = f"Method: {detection_method}"
            cv2.putText(frame, method_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 20
        
        # 目标偏移信息
        if self.display_detection_info:
            target_info = tracker_state.get('target_info', {})
            offset = target_info.get('offset', [0, 0])
            offset_x, offset_y = offset
            offset_text = f"Target Offset: X={offset_x:+.0f}, Y={offset_y:+.0f} px"
            cv2.putText(frame, offset_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20
        
        # 舵机状态
        if self.display_servo_status:
            servo_info = tracker_state.get('servo_info', {})
            x_angle = servo_info.get('x_angle', 0)
            y_angle = servo_info.get('y_angle', 0)
            servo_moving = servo_info.get('moving', False)
            
            servo_text = f"Servo: X={x_angle:.1f}°, Y={y_angle:.1f}°"
            servo_color = (255, 165, 0) if servo_moving else (0, 255, 255)
            cv2.putText(frame, servo_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, servo_color, 1)
            y_offset += 20
            
            if servo_moving:
                moving_text = "Servo: MOVING..."
                cv2.putText(frame, moving_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)
                y_offset += 20
        
        # 映射统计
        if self.display_mapping_stats:
            mapping_info = tracker_state.get('mapping_info', {})
            success_rate = mapping_info.get('success_rate', 0)
            total_mappings = mapping_info.get('total_conversions', 0)
            
            if total_mappings > 0:
                mapping_text = f"Mapping: {success_rate:.1f}% ({total_mappings} total)"
            else:
                mapping_text = f"Mapping: N/A (0 total)"
            cv2.putText(frame, mapping_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20
        
        # 极性状态
        polarity_info = tracker_state.get('polarity_info', {})
        if polarity_info:
            x_desc = polarity_info.get('x_description', 'Normal')
            y_desc = polarity_info.get('y_description', 'Normal')
            polarity_text = f"Polarity: X={x_desc}, Y={y_desc}"
            cv2.putText(frame, polarity_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 20
        
        # 速度信息
        speed_info = tracker_state.get('speed_info', {})
        if speed_info:
            adaptive_speed = speed_info.get('adaptive', False)
            current_speed = speed_info.get('current_speed', 0)
            
            if adaptive_speed:
                speed_text = f"Speed: Adaptive (Base={current_speed})"
            else:
                speed_text = f"Speed: Fixed ({current_speed})"
            cv2.putText(frame, speed_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 20
        
        # 激光状态
        laser_info = tracker_state.get('laser_info', {})
        if laser_info:
            laser_state = laser_info.get('state', False)
            laser_count = laser_info.get('shot_count', 0)
            laser_color = (0, 0, 255) if laser_state else (128, 128, 128)
            laser_text = f"Laser: {'FIRING' if laser_state else 'STANDBY'} ({laser_count} shots)"
            cv2.putText(frame, laser_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, laser_color, 1)
            y_offset += 20
        
        # 控制提示
        control_text = "Controls: SPACE=Track, S=Status, X/Y=Polarity, +/-=Speed, A=Adaptive, D=Display"
        cv2.putText(frame, control_text, (10, frame.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # 统计信息
        stats_info = tracker_state.get('stats_info', {})
        frame_count = stats_info.get('frame_count', 0)
        detection_count = stats_info.get('detection_count', 0)
        control_count = stats_info.get('control_count', 0)
        
        stats_text = f"Frames: {frame_count}, Detections: {detection_count}, Controls: {control_count}"
        cv2.putText(frame, stats_text, (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def initialize_display(self):
        """初始化显示窗口 - 添加缺失的方法"""
        if not self.enabled:
            logger.info("图像显示已禁用，跳过窗口初始化")
            return False
        
        if not self.window_created:
            self._create_window()
        
        if self.window_created:
            logger.info(f"✓ 图像显示初始化完成: {self.window_name}")
            return True
        else:
            logger.error("图像显示窗口创建失败")
            return False
    
    def display_frame(self, frame):
        """显示图像帧并返回按键输入"""
        if not self.enabled or not self.window_created:
            return -1
        
        try:
            # 更新FPS
            self.update_fps()
            
            # 显示图像
            cv2.imshow(self.window_name, frame)
            
            # 获取按键输入
            return cv2.waitKey(1) & 0xFF
            
        except Exception as e:
            logger.error(f"显示图像失败: {e}")
            return -1
    
    def get_key(self, wait_time=1):
        """获取键盘输入 - 添加方法别名"""
        return self.get_key_input(wait_time)
    
    def get_key_input(self, wait_time=1):
        """获取键盘输入"""
        if not self.enabled:
            return -1
        
        try:
            return cv2.waitKey(wait_time) & 0xFF
        except Exception as e:
            logger.error(f"获取键盘输入失败: {e}")
            return -1
    
    def toggle_display_mode(self):
        """切换显示模式"""
        if self.display_detection_info:
            self.display_detection_info = False
            self.display_servo_status = True
            logger.info("🖥️ 显示模式: 隐藏检测信息")
        elif self.display_servo_status:
            self.display_servo_status = False
            self.display_mapping_stats = True
            logger.info("🖥️ 显示模式: 隐藏舵机状态")
        elif self.display_mapping_stats:
            self.display_mapping_stats = False
            self.display_fps_counter = True
            logger.info("🖥️ 显示模式: 隐藏映射统计")
        elif self.display_fps_counter:
            self.display_fps_counter = False
            logger.info("🖥️ 显示模式: 最小化显示")
        else:
            # 重置为全部显示
            self.display_detection_info = True
            self.display_servo_status = True
            self.display_mapping_stats = True
            self.display_fps_counter = True
            logger.info("🖥️ 显示模式: 完整显示")
    
    def show_fps_stats(self, tracker_state: dict):
        """显示FPS统计信息"""
        logger.info("="*50)
        logger.info("FPS 和性能统计:")
        logger.info(f"  当前帧率: {self.current_fps:.1f} fps")
        
        stats_info = tracker_state.get('stats_info', {})
        frame_count = stats_info.get('frame_count', 0)
        detection_count = stats_info.get('detection_count', 0)
        control_count = stats_info.get('control_count', 0)
        
        logger.info(f"  处理帧数: {frame_count}")
        logger.info(f"  检测成功: {detection_count}")
        logger.info(f"  控制次数: {control_count}")
        
        # 计算效率
        if frame_count > 0:
            detection_rate = detection_count / frame_count * 100
            control_rate = control_count / frame_count * 100
            logger.info(f"  检测效率: {detection_rate:.1f}%")
            logger.info(f"  控制效率: {control_rate:.1f}%")
        
        # 映射统计
        mapping_info = tracker_state.get('mapping_info', {})
        success_rate = mapping_info.get('success_rate', 0)
        total_conversions = mapping_info.get('total_conversions', 0)
        
        if total_conversions > 0:
            logger.info(f"  映射成功率: {success_rate:.1f}%")
        
        logger.info("="*50)
    
    def cleanup(self):
        """清理UI资源"""
        if self.enabled and self.window_created:
            try:
                cv2.destroyAllWindows()
                logger.info("UI窗口已清理")
            except Exception as e:
                logger.error(f"清理UI窗口失败: {e}")
    
    def is_enabled(self):
        """检查UI是否启用"""
        return self.enabled
    
    def get_window_name(self):
        """获取窗口名称"""
        return self.window_name if self.enabled else None

class UIStateManager:
    """UI状态管理器 - 收集和组织显示所需的状态信息"""
    
    @staticmethod
    def create_tracker_state(tracker):
        """从跟踪器对象创建状态字典"""
        try:
            # 基本状态
            state = {
                'is_tracking': getattr(tracker, 'is_tracking', False),
                'target_found': getattr(tracker, 'target_found', False),
                'detection_method': getattr(tracker, 'last_detection_method', 'Unknown'),
            }
            
            # 目标信息
            if hasattr(tracker, 'coordinate_mapper'):
                target_info = tracker.coordinate_mapper.get_target_info()
                state['target_info'] = target_info
            else:
                state['target_info'] = {
                    'target_center': [320, 240],
                    'image_center': [320, 240],
                    'offset': [0, 0]
                }
            
            # 舵机信息
            if hasattr(tracker, 'current_x_position'):
                x_angle = tracker.current_x_position * tracker.angle_per_unit
                y_angle = tracker.current_y_position * tracker.angle_per_unit
                state['servo_info'] = {
                    'x_angle': x_angle,
                    'y_angle': y_angle,
                    'moving': getattr(tracker, 'servo_moving', False)
                }
            
            # 映射信息
            if hasattr(tracker, 'coordinate_mapper'):
                mapping_stats = tracker.coordinate_mapper.get_statistics()
                state['mapping_info'] = mapping_stats
            
            # 极性信息
            if hasattr(tracker, 'get_current_polarity_status'):
                state['polarity_info'] = tracker.get_current_polarity_status()
            
            # 速度信息
            if hasattr(tracker, 'adaptive_speed'):
                state['speed_info'] = {
                    'adaptive': tracker.adaptive_speed,
                    'current_speed': getattr(tracker, 'servo_speed', 0)
                }
            
            # 激光信息
            if hasattr(tracker, 'laser_current_state'):
                state['laser_info'] = {
                    'state': tracker.laser_current_state,
                    'shot_count': getattr(tracker, 'laser_shot_count', 0)
                }
            
            # 统计信息
            state['stats_info'] = {
                'frame_count': getattr(tracker, 'frame_count', 0),
                'detection_count': getattr(tracker, 'detection_count', 0),
                'control_count': getattr(tracker, 'control_count', 0)
            }
            
            return state
            
        except Exception as e:
            logger.error(f"创建跟踪器状态失败: {e}")
            # 返回默认状态
            return {
                'is_tracking': False,
                'target_found': False,
                'detection_method': 'Error',
                'target_info': {'target_center': [320, 240], 'image_center': [320, 240], 'offset': [0, 0]},
                'servo_info': {'x_angle': 0, 'y_angle': 0, 'moving': False},
                'mapping_info': {'success_rate': 0, 'total_conversions': 0},
                'stats_info': {'frame_count': 0, 'detection_count': 0, 'control_count': 0}
            }
