import cv2
import numpy as np
import logging
import time
from typing import Dict, Optional, Tuple, List

from YOLO11_Detect_YUV420SP import VisionDetector
from cv_chuli import SquareRefinement, create_refinement_processor

# 配置日志
logger = logging.getLogger("VisionPipeline")

class VisionPipeline:
    """视觉处理管道 - 纯YOLO检测方案"""
    
    def __init__(self, config: Dict):
        """
        初始化视觉处理管道 - 纯检测模式
        :param config: 配置字典
        """
        self.config = config
        
        # 初始化YOLO检测器
        logger.info("初始化YOLO11检测器...")
        self.yolo_detector = VisionDetector(
            model_path=config['yolo']['model_path'],
            conf_thresh=config['yolo']['conf_thresh'],
            iou_thresh=config['yolo']['iou_thresh'],
            min_aspect_ratio=config['yolo'].get('min_aspect_ratio', 0.5),
            max_aspect_ratio=config['yolo'].get('max_aspect_ratio', 2.0)
        )
        
        # 禁用OpenCV精细化处理
        logger.info("OpenCV精细化处理已禁用 - 使用纯YOLO检测方案")
        self.opencv_processor = None
        
        # 处理模式配置 - 强制使用纯检测模式
        self.enable_opencv_refinement = False  # 强制禁用
        self.fallback_to_yolo = True
        
        # 不需要扩展检测框
        self.bbox_expansion = 0
        
        # 性能统计
        self.stats = {
            'total_frames': 0,
            'yolo_detections': 0,
            'aspect_ratio_filtered': 0,
            'opencv_successes': 0,      # 始终为0
            'opencv_failures': 0,       # 始终为0
            'fallback_uses': 0,         # 始终为0
            'total_yolo_time': 0.0,
            'total_opencv_time': 0.0    # 始终为0
        }
        
        logger.info(f"纯YOLO检测管道初始化完成")
        logger.info(f"长宽比筛选: {config['yolo'].get('min_aspect_ratio', 0.5):.1f} - {config['yolo'].get('max_aspect_ratio', 2.0):.1f}")
        logger.info(f"🚀 性能优化: 跳过OpenCV处理，直接使用YOLO检测框中心点")

    def process_frame(self, frame: np.ndarray, enable_debug: bool = False) -> Dict:
        """
        处理单帧图像 - 纯YOLO检测方案
        :param frame: 输入图像
        :param enable_debug: 是否启用调试信息（在纯检测模式下基本无用）
        :return: 处理结果字典
        """
        self.stats['total_frames'] += 1
        result = {
            'success': False,
            'center': None,
            'bbox': None,
            'confidence': 0.0,
            'method': 'pure_yolo',
            'yolo_result': None,
            'opencv_result': None,       # 始终为None
            'expanded_bbox': None,       # 不再需要
            'processing_time': {
                'yolo': 0.0,
                'opencv': 0.0,           # 始终为0
                'total': 0.0
            }
        }
        
        start_time = time.time()
        
        try:
            # 纯YOLO检测
            yolo_start = time.time()
            yolo_result = self.yolo_detector.get_best_detection(frame)
            yolo_time = time.time() - yolo_start
            
            self.stats['total_yolo_time'] += yolo_time
            result['processing_time']['yolo'] = yolo_time
            result['yolo_result'] = yolo_result
            
            if not yolo_result['found']:
                result['method'] = 'yolo_no_detection'
                result['processing_time']['total'] = time.time() - start_time
                return result
            
            self.stats['yolo_detections'] += 1
            
            # 直接使用YOLO结果，无需OpenCV处理
            result.update({
                'success': True,
                'center': yolo_result['center'],
                'bbox': yolo_result['bbox'],
                'confidence': yolo_result['confidence'],
                'method': 'pure_yolo_detection',
                'size': yolo_result.get('size', [0, 0]),
                'aspect_ratio': yolo_result.get('aspect_ratio', 0)
            })
            
            if enable_debug:
                logger.debug(f"纯YOLO检测成功: 中心点({yolo_result['center'][0]}, {yolo_result['center'][1]}), 置信度: {yolo_result['confidence']:.2f}")
            
            result['processing_time']['total'] = time.time() - start_time
            return result
            
        except Exception as e:
            logger.error(f"纯YOLO检测失败: {e}")
            result.update({
                'method': 'error',
                'error': str(e),
                'processing_time': {'total': time.time() - start_time}
            })
            return result

    def get_processing_stats(self) -> Dict:
        """获取处理统计信息 - 纯检测版本"""
        if self.stats['total_frames'] > 0:
            yolo_avg_time = (self.stats['total_yolo_time'] / self.stats['total_frames']) * 1000
            
            stats = {
                'total_frames': self.stats['total_frames'],
                'yolo_detection_rate': (self.stats['yolo_detections'] / self.stats['total_frames']) * 100,
                'aspect_ratio_filter_rate': (self.stats['aspect_ratio_filtered'] / max(self.stats['yolo_detections'], 1)) * 100,
                'opencv_success_rate': 0.0,    # 始终为0
                'opencv_failure_rate': 0.0,    # 始终为0
                'fallback_rate': 0.0,          # 始终为0
                'avg_yolo_time_ms': yolo_avg_time,
                'avg_opencv_time_ms': 0.0,     # 始终为0
                'total_processing_time_ms': yolo_avg_time  # 只有YOLO时间
            }
            return stats
        return {}

    def set_opencv_enabled(self, enabled: bool):
        """动态启用/禁用OpenCV精细化处理 - 在纯检测模式下无效"""
        if enabled:
            logger.warning("纯YOLO检测模式下无法启用OpenCV精细化处理")
        else:
            logger.info("OpenCV精细化处理已禁用（纯检测模式）")

    def visualize_result(self, frame: np.ndarray, result: Dict) -> np.ndarray:
        """
        可视化处理结果 - 纯YOLO版本
        :param frame: 原始图像
        :param result: 处理结果
        :return: 可视化后的图像
        """
        if not result['success']:
            return frame
        
        vis_frame = frame.copy()
        
        try:
            # 绘制YOLO检测结果
            center = result['center']
            bbox = result['bbox']
            method = result['method']
            
            # 使用蓝色表示纯YOLO检测
            color = (255, 0, 0)  # 蓝色 - 纯YOLO检测
            thickness = 3
            
            # 绘制边界框
            cv2.rectangle(vis_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
            
            # 绘制中心点
            cv2.circle(vis_frame, tuple(center), 10, color, -1)
            cv2.circle(vis_frame, tuple(center), 15, color, 3)
            
            # 绘制十字线
            cv2.line(vis_frame, (center[0]-20, center[1]), (center[0]+20, center[1]), color, 3)
            cv2.line(vis_frame, (center[0], center[1]-20), (center[0], center[1]+20), color, 3)
            
            # 显示置信度信息
            info_text = f"PURE YOLO: {result['confidence']:.2f}"
            cv2.putText(vis_frame, info_text, (bbox[0], bbox[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 显示尺寸和长宽比信息
            if 'size' in result and 'aspect_ratio' in result:
                size_text = f"Size: {result['size'][0]}x{result['size'][1]}, Ratio: {result['aspect_ratio']:.2f}"
                cv2.putText(vis_frame, size_text, (bbox[0], bbox[1]-35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # 显示处理时间信息
            time_info = result['processing_time']
            time_text = f"YOLO: {time_info['yolo']*1000:.1f}ms (Pure Detection)"
            cv2.putText(vis_frame, time_text, (10, vis_frame.shape[0]-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 添加纯检测模式标识
            mode_text = "PURE YOLO DETECTION MODE"
            cv2.putText(vis_frame, mode_text, (10, vis_frame.shape[0]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        except Exception as e:
            logger.error(f"可视化失败: {e}")
        
        return vis_frame

class VisionConfig:
    """视觉处理配置管理器 - 纯检测版本"""
    
    @staticmethod
    def create_default_config() -> Dict:
        """创建默认视觉处理配置 - 纯检测模式"""
        return {
            # YOLO配置
            'yolo': {
                'model_path': '/root/square/detect/2.0.bin',
                'conf_thresh': 0.25,
                'iou_thresh': 0.45,
                'min_aspect_ratio': 0.5,
                'max_aspect_ratio': 2.0
            },
            
            # OpenCV精细化配置 - 已禁用
            'opencv': {
                'min_contour_area': 300,
                'max_contour_area': 15000,
                'blur_kernel_size': 5,
                'morph_kernel_size': 3
            },
            
            # 处理流程配置 - 纯检测模式
            'enable_opencv_refinement': False,  # 强制禁用
            'fallback_to_yolo': True,
            'bbox_expansion': 0,                # 不需要扩展
            
            # 摄像头配置
            'camera': {
                'width': 640,
                'height': 480,
                'fps': 30
            }
        }
    
    @staticmethod
    def create_performance_config() -> Dict:
        """创建性能优先配置 - 纯检测模式（与默认相同）"""
        return VisionConfig.create_default_config()
    
    @staticmethod
    def create_accuracy_config() -> Dict:
        """创建精度优先配置 - 纯检测模式（与默认相同）"""
        return VisionConfig.create_default_config()

if __name__ == "__main__":
    # 视觉处理管道测试 - 纯检测版本
    import sys
    import time
    
    print("启动纯YOLO检测管道测试...")
    print("=" * 50)
    print("处理流程: YOLO11检测 → 直接使用检测框中心点")
    print("🚀 性能优化: 跳过OpenCV后处理")
    print("=" * 50)
    
    try:
        # 创建配置
        config = VisionConfig.create_default_config()
        
        # 初始化视觉管道
        pipeline = VisionPipeline(config)
        
        # 连接摄像头
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['camera']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['camera']['height'])
        cap.set(cv2.CAP_PROP_FPS, config['camera']['fps'])
        
        print("\n按键说明:")
        print("'q' - 退出")
        print("'s' - 显示处理统计")
        print("'r' - 重置统计")
        print("'v' - 切换可视化显示")
        
        show_visualization = True
        frame_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头帧")
                break
            
            frame_count += 1
            
            # 处理帧
            result = pipeline.process_frame(frame, enable_debug=(frame_count % 30 == 0))
            
            # 可视化
            if show_visualization:
                vis_frame = pipeline.visualize_result(frame, result)
                
                # 添加状态信息
                status_text = f"Frame: {frame_count}, Method: {result['method']}"
                cv2.putText(vis_frame, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if result['success']:
                    center_text = f"Center: {result['center']}"
                    cv2.putText(vis_frame, center_text, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.imshow("Pure YOLO Detection Pipeline Test", vis_frame)
            
            # 控制台输出
            if frame_count % 10 == 0:
                if result['success']:
                    center = result['center']
                    method = result['method']
                    total_time = result['processing_time']['total'] * 1000
                    print(f"\r帧{frame_count}: {method} -> 中心点({center[0]}, {center[1]}) 耗时{total_time:.1f}ms [纯检测]", end="", flush=True)
                else:
                    print(f"\r帧{frame_count}: 未检测到目标", end="", flush=True)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('v'):
                show_visualization = not show_visualization
                print(f"\n可视化显示: {'开启' if show_visualization else '关闭'}")
            elif key == ord('s'):
                stats = pipeline.get_processing_stats()
                if stats:
                    print("\n" + "="*60)
                    print("纯YOLO检测统计:")
                    for key, value in stats.items():
                        if 'rate' in key or 'time' in key:
                            print(f"  {key}: {value:.2f}")
                        else:
                            print(f"  {key}: {value}")
                    print("="*60)
            elif key == ord('r'):
                pipeline.reset_stats()
                frame_count = 0
                print("\n统计信息已重置")

    except Exception as e:
        print(f"测试程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'cap' in locals():
                cap.release()
            cv2.destroyAllWindows()
            
            # 最终统计
            if 'pipeline' in locals():
                final_stats = pipeline.get_processing_stats()
                if final_stats:
                    print("\n" + "="*60)
                    print("最终纯YOLO检测统计:")
                    for key, value in final_stats.items():
                        if 'rate' in key or 'time' in key:
                            print(f"  {key}: {value:.2f}")
                        else:
                            print(f"  {key}: {value}")
                    print("="*60)
            
            print("纯检测测试完成")
        except:
            pass
