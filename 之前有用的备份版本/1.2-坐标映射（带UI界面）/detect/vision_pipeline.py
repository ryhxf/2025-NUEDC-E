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
    """视觉处理管道 - 整合YOLO11检测和OpenCV精细化处理"""
    
    def __init__(self, config: Dict):
        """
        初始化视觉处理管道
        :param config: 配置字典
        """
        self.config = config
        
        # 初始化YOLO检测器
        logger.info("初始化YOLO11检测器...")
        self.yolo_detector = VisionDetector(
            model_path=config['yolo']['model_path'],
            conf_thresh=config['yolo']['conf_thresh'],
            iou_thresh=config['yolo']['iou_thresh']
        )
        
        # 初始化OpenCV精细化处理器
        logger.info("初始化OpenCV精细化处理器...")
        opencv_config = config.get('opencv', {})
        self.opencv_processor = create_refinement_processor(opencv_config)
        
        # 处理模式配置
        self.enable_opencv_refinement = config.get('enable_opencv_refinement', True)
        self.fallback_to_yolo = config.get('fallback_to_yolo', True)
        
        # 性能统计
        self.stats = {
            'total_frames': 0,
            'yolo_detections': 0,
            'opencv_successes': 0,
            'opencv_failures': 0,
            'fallback_uses': 0,
            'total_yolo_time': 0.0,
            'total_opencv_time': 0.0
        }
        
        logger.info(f"视觉处理管道初始化完成 - OpenCV精细化: {'启用' if self.enable_opencv_refinement else '禁用'}")
    
    def process_frame(self, frame: np.ndarray, enable_debug: bool = False) -> Dict:
        """
        处理单帧图像，返回精确的中心点
        :param frame: 输入图像
        :param enable_debug: 是否启用调试信息
        :return: 处理结果字典
        """
        self.stats['total_frames'] += 1
        result = {
            'success': False,
            'center': None,
            'bbox': None,
            'confidence': 0.0,
            'method': 'none',
            'yolo_result': None,
            'opencv_result': None,
            'processing_time': {
                'yolo': 0.0,
                'opencv': 0.0,
                'total': 0.0
            }
        }
        
        start_time = time.time()
        
        try:
            # 第一步：YOLO检测
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
            
            # 第二步：OpenCV精细化处理（可选）
            if self.enable_opencv_refinement:
                opencv_start = time.time()
                opencv_result = self.opencv_processor.refine_detection_in_bbox(
                    frame, yolo_result['bbox'], show_debug=enable_debug
                )
                opencv_time = time.time() - opencv_start
                
                self.stats['total_opencv_time'] += opencv_time
                result['processing_time']['opencv'] = opencv_time
                result['opencv_result'] = opencv_result
                
                if opencv_result['success']:
                    # OpenCV处理成功，使用精细化结果
                    self.stats['opencv_successes'] += 1
                    result.update({
                        'success': True,
                        'center': opencv_result['center'],
                        'bbox': opencv_result['bbox'],
                        'confidence': yolo_result['confidence'],  # 保持YOLO的置信度
                        'method': f"opencv_{opencv_result['method']}",
                        'rectangle_points': opencv_result.get('rectangle_points'),
                        'improvement': opencv_result.get('improvement', 0)
                    })
                    
                    if enable_debug:
                        logger.debug(f"OpenCV精细化成功: {opencv_result['method']}, 中心点偏移: {self._calculate_center_offset(yolo_result['center'], opencv_result['center']):.1f}px")
                
                else:
                    # OpenCV处理失败
                    self.stats['opencv_failures'] += 1
                    
                    if self.fallback_to_yolo:
                        # 回退到YOLO结果
                        self.stats['fallback_uses'] += 1
                        result.update({
                            'success': True,
                            'center': yolo_result['center'],
                            'bbox': yolo_result['bbox'],
                            'confidence': yolo_result['confidence'],
                            'method': 'yolo_fallback'
                        })
                        
                        if enable_debug:
                            logger.debug(f"OpenCV处理失败，回退到YOLO: {opencv_result.get('debug_info', '未知原因')}")
                    else:
                        result['method'] = 'opencv_failed'
            
            else:
                # 不使用OpenCV精细化，直接使用YOLO结果
                result.update({
                    'success': True,
                    'center': yolo_result['center'],
                    'bbox': yolo_result['bbox'],
                    'confidence': yolo_result['confidence'],
                    'method': 'yolo_only'
                })
            
            result['processing_time']['total'] = time.time() - start_time
            return result
            
        except Exception as e:
            logger.error(f"视觉处理管道异常: {e}")
            result.update({
                'method': 'error',
                'error': str(e),
                'processing_time': {'total': time.time() - start_time}
            })
            return result
    
    def _calculate_center_offset(self, yolo_center: List[int], opencv_center: List[int]) -> float:
        """计算YOLO和OpenCV中心点的偏移距离"""
        dx = opencv_center[0] - yolo_center[0]
        dy = opencv_center[1] - yolo_center[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def get_processing_stats(self) -> Dict:
        """获取处理统计信息"""
        if self.stats['total_frames'] > 0:
            yolo_avg_time = (self.stats['total_yolo_time'] / self.stats['total_frames']) * 1000
            opencv_avg_time = (self.stats['total_opencv_time'] / max(self.stats['yolo_detections'], 1)) * 1000
            
            return {
                'total_frames': self.stats['total_frames'],
                'yolo_detection_rate': (self.stats['yolo_detections'] / self.stats['total_frames']) * 100,
                'opencv_success_rate': (self.stats['opencv_successes'] / max(self.stats['yolo_detections'], 1)) * 100,
                'opencv_failure_rate': (self.stats['opencv_failures'] / max(self.stats['yolo_detections'], 1)) * 100,
                'fallback_rate': (self.stats['fallback_uses'] / max(self.stats['yolo_detections'], 1)) * 100,
                'avg_yolo_time_ms': yolo_avg_time,
                'avg_opencv_time_ms': opencv_avg_time,
                'total_processing_time_ms': yolo_avg_time + opencv_avg_time
            }
        return {}
    
    def reset_stats(self):
        """重置统计信息"""
        self.stats = {
            'total_frames': 0,
            'yolo_detections': 0,
            'opencv_successes': 0,
            'opencv_failures': 0,
            'fallback_uses': 0,
            'total_yolo_time': 0.0,
            'total_opencv_time': 0.0
        }
        logger.info("视觉处理统计已重置")
    
    def set_opencv_enabled(self, enabled: bool):
        """动态启用/禁用OpenCV精细化处理"""
        self.enable_opencv_refinement = enabled
        logger.info(f"OpenCV精细化处理: {'启用' if enabled else '禁用'}")
    
    def visualize_result(self, frame: np.ndarray, result: Dict) -> np.ndarray:
        """
        可视化处理结果
        :param frame: 原始图像
        :param result: 处理结果
        :return: 可视化后的图像
        """
        if not result['success']:
            return frame
        
        vis_frame = frame.copy()
        
        try:
            # 绘制最终结果
            center = result['center']
            bbox = result['bbox']
            method = result['method']
            
            # 根据处理方法选择颜色
            if 'opencv' in method:
                color = (0, 255, 0)  # 绿色 - OpenCV处理
                thickness = 3
            elif 'yolo' in method:
                color = (0, 255, 255)  # 黄色 - YOLO处理
                thickness = 2
            else:
                color = (128, 128, 128)  # 灰色 - 其他
                thickness = 1
            
            # 绘制边界框
            cv2.rectangle(vis_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
            
            # 绘制中心点
            cv2.circle(vis_frame, tuple(center), 8, color, -1)
            cv2.circle(vis_frame, tuple(center), 12, color, 2)
            
            # 绘制十字线
            cv2.line(vis_frame, (center[0]-15, center[1]), (center[0]+15, center[1]), color, 2)
            cv2.line(vis_frame, (center[0], center[1]-15), (center[0], center[1]+15), color, 2)
            
            # 显示方法和置信度信息
            info_text = f"{method}: {result['confidence']:.2f}"
            cv2.putText(vis_frame, info_text, (bbox[0], bbox[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 如果有矩形角点，绘制精确的四边形
            if 'rectangle_points' in result and result['rectangle_points']:
                points = result['rectangle_points']
                points_np = np.array(points, dtype=np.int32)
                cv2.polylines(vis_frame, [points_np], True, (0, 255, 0), 2)
                
                # 绘制角点编号
                for i, point in enumerate(points):
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(vis_frame, (x, y), 4, (0, 255, 0), -1)
                    cv2.putText(vis_frame, str(i), (x+5, y-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # 显示处理时间信息
            time_info = result['processing_time']
            time_text = f"YOLO: {time_info['yolo']*1000:.1f}ms, OpenCV: {time_info.get('opencv', 0)*1000:.1f}ms"
            cv2.putText(vis_frame, time_text, (10, vis_frame.shape[0]-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示改进信息
            if 'improvement' in result:
                improve_text = f"Improvement: {result['improvement']:.1f}%"
                cv2.putText(vis_frame, improve_text, (10, vis_frame.shape[0]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        except Exception as e:
            logger.error(f"可视化失败: {e}")
        
        return vis_frame

class VisionConfig:
    """视觉处理配置管理器"""
    
    @staticmethod
    def create_default_config() -> Dict:
        """创建默认视觉处理配置"""
        return {
            # YOLO配置
            'yolo': {
                'model_path': '/root/square/detect/1.0.bin',
                'conf_thresh': 0.25,
                'iou_thresh': 0.45
            },
            
            # OpenCV精细化配置
            'opencv': {
                'min_contour_area': 100,
                'max_contour_area': 10000,
                'blur_kernel_size': 5,
                'morph_kernel_size': 3
            },
            
            # 处理流程配置
            'enable_opencv_refinement': True,  # 是否启用OpenCV精细化
            'fallback_to_yolo': True,          # OpenCV失败时是否回退到YOLO
            
            # 摄像头配置
            'camera': {
                'width': 640,
                'height': 480,
                'fps': 30
            }
        }
    
    @staticmethod
    def create_performance_config() -> Dict:
        """创建性能优先配置（禁用OpenCV）"""
        config = VisionConfig.create_default_config()
        config.update({
            'enable_opencv_refinement': False,
            'fallback_to_yolo': True
        })
        return config
    
    @staticmethod
    def create_accuracy_config() -> Dict:
        """创建精度优先配置（启用所有处理）"""
        config = VisionConfig.create_default_config()
        config.update({
            'enable_opencv_refinement': True,
            'fallback_to_yolo': True
        })
        return config

if __name__ == "__main__":
    # 视觉处理管道测试
    import sys
    import time
    
    print("启动视觉处理管道测试...")
    print("=" * 50)
    print("处理流程: YOLO11检测 → OpenCV精细化 → 精确中心点")
    print("=" * 50)
    
    try:
        # 创建配置
        config = VisionConfig.create_accuracy_config()
        
        # 初始化视觉管道
        pipeline = VisionPipeline(config)
        
        # 连接摄像头
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['camera']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['camera']['height'])
        cap.set(cv2.CAP_PROP_FPS, config['camera']['fps'])
        
        print("\n按键说明:")
        print("'q' - 退出")
        print("'o' - 切换OpenCV精细化开关")
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
                
                cv2.imshow("Vision Pipeline Test", vis_frame)
            
            # 控制台输出
            if frame_count % 10 == 0:
                if result['success']:
                    center = result['center']
                    method = result['method']
                    total_time = result['processing_time']['total'] * 1000
                    print(f"\r帧{frame_count}: {method} -> 中心点({center[0]}, {center[1]}) 耗时{total_time:.1f}ms", end="", flush=True)
                else:
                    print(f"\r帧{frame_count}: 未检测到目标", end="", flush=True)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('o'):
                current_state = pipeline.enable_opencv_refinement
                pipeline.set_opencv_enabled(not current_state)
                print(f"\nOpenCV精细化: {'启用' if not current_state else '禁用'}")
            elif key == ord('s'):
                stats = pipeline.get_processing_stats()
                print("\n" + "="*60)
                print("视觉处理统计:")
                for key, value in stats.items():
                    if 'rate' in key or 'time' in key:
                        print(f"  {key}: {value:.2f}")
                    else:
                        print(f"  {key}: {value}")
                print("="*60)
            elif key == ord('r'):
                pipeline.reset_stats()
                frame_count = 0
                print("\n统计已重置")
            elif key == ord('v'):
                show_visualization = not show_visualization
                if not show_visualization:
                    cv2.destroyAllWindows()
                print(f"\n可视化显示: {'开启' if show_visualization else '关闭'}")
    
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
                    print("最终处理统计:")
                    for key, value in final_stats.items():
                        if 'rate' in key or 'time' in key:
                            print(f"  {key}: {value:.2f}")
                        else:
                            print(f"  {key}: {value}")
                    print("="*60)
            
            print("测试完成")
        except:
            pass
