import cv2
import numpy as np
import logging
from collections import deque

# 配置日志
logger = logging.getLogger("OpenCV_PostProcess")

class SquareRefinement:
    """方块检测精细化处理模块 - 已禁用，保持接口兼容性"""
    
    def __init__(self, 
                 min_contour_area: int = 50,
                 max_contour_area: int = 15000,
                 blur_kernel_size: int = 5,
                 morph_kernel_size: int = 3):
        """初始化方块精细化检测器 - 已禁用"""
        self.min_contour_area = min_contour_area
        self.max_contour_area = max_contour_area
        self.blur_kernel_size = blur_kernel_size
        self.morph_kernel_size = morph_kernel_size
        
        # 禁用标志
        self.disabled = True
        
        logger.info("OpenCV精细化处理已禁用 - 使用纯YOLO检测方案")

    def find_largest_rectangle_in_roi(self, roi_image: np.ndarray, show_debug_windows: bool = False, yolo_center: tuple = None) -> dict:
        """
        已禁用 - 直接返回失败
        """
        return {'success': False, 'reason': 'OpenCV精细化处理已禁用'}

    def refine_detection_in_bbox(self, frame: np.ndarray, yolo_bbox: list, show_debug: bool = False) -> dict:
        """已禁用 - 直接返回失败"""
        return self._create_failed_result("OpenCV精细化处理已禁用")

    def _create_failed_result(self, reason: str = "已禁用") -> dict:
        """创建失败结果"""
        return {
            'success': False,
            'center': None,
            'bbox': None,
            'area': 0,
            'improvement': 0,
            'debug_info': f"失败原因: {reason}"
        }

def create_refinement_processor(config: dict = None) -> SquareRefinement:
    """创建精细化处理器的工厂函数 - 返回禁用的处理器"""
    default_config = {
        'min_contour_area': 100,
        'max_contour_area': 10000,
        'blur_kernel_size': 5,
        'morph_kernel_size': 3
    }
    
    if config:
        default_config.update(config)
    
    return SquareRefinement(**default_config)
