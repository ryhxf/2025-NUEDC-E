import cv2
import numpy as np
import logging

# 配置日志
logger = logging.getLogger("OpenCV_PostProcess")

class SquareRefinement:
    """方块检测精细化处理模块 - 外围四边形检测"""
    
    def __init__(self, 
                 min_contour_area: int = 50,
                 max_contour_area: int = 15000,
                 blur_kernel_size: int = 5,
                 morph_kernel_size: int = 3):
        """初始化方块精细化检测器"""
        self.min_contour_area = min_contour_area
        self.max_contour_area = max_contour_area
        self.blur_kernel_size = blur_kernel_size
        self.morph_kernel_size = morph_kernel_size
        
        logger.info(f"改进的矩形检测器初始化完成")

    def find_largest_rectangle_in_roi(self, roi_image: np.ndarray, show_debug_windows: bool = False) -> dict:
        """
        在ROI图像中检测黑色区域的外围四边形
        """
        try:
            if roi_image.size == 0:
                return {'success': False, 'reason': 'ROI图像为空'}
            
            # 转为灰度图
            if len(roi_image.shape) == 3:
                gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
            else:
                gray = roi_image.copy()
            
            # 主要方法：寻找黑色区域的外围四边形
            outer_quad_result = self._find_outer_quadrilateral(gray)
            
            # 备选方法：改进的霍夫直线检测
            hough_result = self._improved_hough_detection(gray)
            
            # 选择最佳方法
            best_result = self._select_best_method(outer_quad_result, hough_result)
            
            # 调试窗口
            if show_debug_windows:
                self._show_debug_window(roi_image, outer_quad_result, hough_result, best_result)
            
            return best_result
            
        except Exception as e:
            logger.error(f"四边形检测失败: {e}")
            return {'success': False, 'reason': f'处理异常: {str(e)}'}
    
    def _find_outer_quadrilateral(self, gray):
        """寻找黑色区域的外围四边形"""
        try:
            # 高斯模糊
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 多种二值化方法，找到黑色区域
            # 方法1: OTSU反向阈值
            _, otsu_thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            # 方法2: 自适应反向阈值
            adaptive_thresh = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
            )
            
            # 方法3: 固定低阈值（专门找黑色区域）
            _, fixed_thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
            
            # 尝试每种方法
            thresh_methods = [
                ('otsu_inv', otsu_thresh),
                ('adaptive_inv', adaptive_thresh),
                ('fixed_inv', fixed_thresh)
            ]
            
            best_result = None
            best_score = 0
            debug_info = []
            
            for method_name, binary_img in thresh_methods:
                # 形态学操作 - 连接断开的区域
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                processed = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel, iterations=3)
                processed = cv2.morphologyEx(processed, cv2.MORPH_OPEN, kernel, iterations=1)
                
                # 寻找轮廓
                contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                debug_info.append((method_name, binary_img, processed, len(contours)))
                
                if not contours:
                    continue
                
                # 分析每个轮廓
                for contour in contours:
                    area = cv2.contourArea(contour)
                    
                    # 面积筛选
                    if area < 200 or area > 15000:
                        continue
                    
                    # 寻找外围四边形角点
                    quad_points = self._find_outer_corner_points(contour, gray.shape)
                    
                    if quad_points is None:
                        continue
                    
                    # 计算四边形面积作为评分
                    quad_area = cv2.contourArea(np.array(quad_points, dtype=np.float32))
                    
                    if quad_area > best_score:
                        best_score = quad_area
                        
                        # 计算中心点
                        center_x = np.mean([p[0] for p in quad_points])
                        center_y = np.mean([p[1] for p in quad_points])
                        
                        # 计算边界框
                        x_coords = [p[0] for p in quad_points]
                        y_coords = [p[1] for p in quad_points]
                        x1, y1 = max(0, int(min(x_coords))), max(0, int(min(y_coords)))
                        x2, y2 = min(gray.shape[1], int(max(x_coords))), min(gray.shape[0], int(max(y_coords)))
                        
                        best_result = {
                            'success': True,
                            'center': [int(center_x), int(center_y)],
                            'bbox': [x1, y1, x2, y2],
                            'rectangle_points': quad_points,
                            'area': quad_area,
                            'contour_area': area,
                            'method': f'outer_quad_{method_name}',
                            'score': quad_area
                        }
            
            if best_result is None:
                return {'success': False, 'reason': f'未找到合适的外围四边形，尝试方法: {[info[0] for info in debug_info]}'}
            
            best_result['debug_info'] = debug_info
            return best_result
            
        except Exception as e:
            logger.error(f"外围四边形检测失败: {e}")
            return {'success': False, 'reason': f'外围四边形检测异常: {str(e)}'}
    
    def _find_outer_corner_points(self, contour, image_shape):
        """寻找轮廓的外围四个角点"""
        try:
            h, w = image_shape
            
            # 方法1: 使用凸包找到最外围的点
            hull = cv2.convexHull(contour)
            
            # 如果凸包点数太少，直接返回
            if len(hull) < 4:
                return None
            
            # 方法2: 找到四个极值点（最左上、最右上、最右下、最左下）
            # 将凸包点转换为2D数组
            hull_points = hull.reshape(-1, 2)
            
            # 找到四个极值点
            # 最左上：x+y最小
            top_left_idx = np.argmin(hull_points[:, 0] + hull_points[:, 1])
            top_left = hull_points[top_left_idx]
            
            # 最右上：x-y最大
            top_right_idx = np.argmax(hull_points[:, 0] - hull_points[:, 1])
            top_right = hull_points[top_right_idx]
            
            # 最右下：x+y最大
            bottom_right_idx = np.argmax(hull_points[:, 0] + hull_points[:, 1])
            bottom_right = hull_points[bottom_right_idx]
            
            # 最左下：y-x最大
            bottom_left_idx = np.argmax(hull_points[:, 1] - hull_points[:, 0])
            bottom_left = hull_points[bottom_left_idx]
            
            # 构建四边形点列表（按顺序：左上、右上、右下、左下）
            quad_points = [
                [float(top_left[0]), float(top_left[1])],
                [float(top_right[0]), float(top_right[1])],
                [float(bottom_right[0]), float(bottom_right[1])],
                [float(bottom_left[0]), float(bottom_left[1])]
            ]
            
            # 验证四边形的有效性
            if self._validate_quadrilateral(quad_points, w, h):
                return quad_points
            
            # 如果极值点方法失败，尝试多边形近似
            return self._approximate_quadrilateral(hull, w, h)
            
        except Exception as e:
            logger.error(f"寻找外围角点失败: {e}")
            return None
    
    def _validate_quadrilateral(self, points, width, height):
        """验证四边形的有效性"""
        try:
            # 检查点是否在图像范围内
            for point in points:
                x, y = point
                if x < -width*0.1 or x > width*1.1 or y < -height*0.1 or y > height*1.1:
                    return False
            
            # 检查四边形面积是否合理
            area = cv2.contourArea(np.array(points, dtype=np.float32))
            if area < 100 or area > width * height * 0.8:
                return False
            
            # 检查是否为退化的四边形（三点共线等）
            if area < 50:
                return False
            
            return True
            
        except:
            return False
    
    def _approximate_quadrilateral(self, contour, width, height):
        """使用多边形近似获得四边形"""
        try:
            # 计算轮廓周长
            perimeter = cv2.arcLength(contour, True)
            
            # 尝试不同的近似精度
            epsilons = [0.02, 0.03, 0.04, 0.05, 0.06]
            
            for epsilon in epsilons:
                # 多边形近似
                approx = cv2.approxPolyDP(contour, epsilon * perimeter, True)
                
                if len(approx) == 4:
                    # 找到了四边形，按顺序排列点
                    points = approx.reshape(-1, 2).astype(float)
                    ordered_points = self._order_quadrilateral_points(points)
                    
                    if self._validate_quadrilateral(ordered_points, width, height):
                        return ordered_points
                
                elif len(approx) > 4:
                    # 如果点数多于4个，选择最外围的4个点
                    points = approx.reshape(-1, 2).astype(float)
                    outer_points = self._select_outer_four_points(points)
                    
                    if outer_points and self._validate_quadrilateral(outer_points, width, height):
                        return outer_points
            
            return None
            
        except Exception as e:
            logger.error(f"多边形近似失败: {e}")
            return None
    
    def _select_outer_four_points(self, points):
        """从多个点中选择最外围的四个点"""
        try:
            if len(points) < 4:
                return None
            
            # 计算中心点
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            # 计算每个点到中心的距离
            distances = np.sqrt((points[:, 0] - center_x)**2 + (points[:, 1] - center_y)**2)
            
            # 选择距离最远的4个点
            far_indices = np.argsort(distances)[-4:]
            outer_points = points[far_indices]
            
            # 按顺序排列
            ordered_points = self._order_quadrilateral_points(outer_points)
            
            return ordered_points
            
        except Exception as e:
            logger.error(f"选择外围四点失败: {e}")
            return None
    
    def _order_quadrilateral_points(self, points):
        """将四个点按照左上、右上、右下、左下的顺序排列"""
        try:
            # 计算中心点
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            # 计算每个点相对于中心的角度
            angles = np.arctan2(points[:, 1] - center_y, points[:, 0] - center_x)
            
            # 按角度排序
            sorted_indices = np.argsort(angles)
            sorted_points = points[sorted_indices]
            
            # 找到左上角点（x+y最小）作为起始点
            sums = sorted_points[:, 0] + sorted_points[:, 1]
            start_idx = np.argmin(sums)
            
            # 重新排列，从左上角开始顺时针
            ordered_points = np.roll(sorted_points, -start_idx, axis=0)
            
            return ordered_points.tolist()
            
        except Exception as e:
            logger.error(f"排序四边形点失败: {e}")
            return points.tolist()
    
    def _improved_hough_detection(self, gray):
        """改进的霍夫直线检测 - 解决粗直线问题"""
        try:
            # 高斯模糊
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # 使用更保守的边缘检测参数
            edges = cv2.Canny(blurred, 30, 90, apertureSize=3)
            
            # 轻微的形态学操作，避免线条过粗
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)
            
            # 霍夫直线检测 - 调整参数减少重复检测
            lines = cv2.HoughLinesP(
                edges,
                rho=2,                    # 增加距离精度，减少相近直线
                theta=np.pi/90,           # 增加角度精度
                threshold=40,             # 提高阈值
                minLineLength=30,         # 增加最小线段长度
                maxLineGap=15             # 增加最大间隙
            )
            
            if lines is None or len(lines) < 4:
                return {'success': False, 'reason': f'霍夫直线数量不足: {len(lines) if lines is not None else 0}'}
            
            # 关键改进：合并相似的直线
            merged_lines = self._merge_similar_lines(lines)
            
            if len(merged_lines) < 4:
                return {'success': False, 'reason': f'合并后直线数量不足: {len(merged_lines)}'}
            
            # 分析合并后的直线构建矩形
            rectangle_result = self._analyze_merged_lines_to_rectangle(merged_lines, gray.shape)
            
            if rectangle_result['success']:
                rectangle_result['method'] = 'improved_hough'
                rectangle_result['original_lines'] = len(lines)
                rectangle_result['merged_lines'] = len(merged_lines)
            
            return rectangle_result
            
        except Exception as e:
            logger.error(f"改进霍夫检测失败: {e}")
            return {'success': False, 'reason': f'霍夫检测异常: {str(e)}'}
    
    def _merge_similar_lines(self, lines):
        """合并相似的直线 - 解决粗直线被检测成多条线的问题"""
        try:
            if len(lines) == 0:
                return []
            
            # 将所有线段转换为角度和距离表示
            line_params = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # 计算线段中点、长度、角度
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                # 计算角度（归一化到0-180度）
                angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
                if angle < 0:
                    angle += 180
                
                # 计算到原点的距离（垂直距离）
                if abs(x2-x1) < 1e-6:  # 垂直线
                    distance = abs(x1)
                else:
                    # 直线方程 ax + by + c = 0
                    a = y2 - y1
                    b = x1 - x2
                    c = x2*y1 - x1*y2
                    distance = abs(c) / np.sqrt(a*a + b*b)
                
                line_params.append({
                    'line': line[0],
                    'center': (center_x, center_y),
                    'length': length,
                    'angle': angle,
                    'distance': distance
                })
            
            # 按角度分组（水平和垂直）
            horizontal_lines = [p for p in line_params if abs(p['angle']) < 30 or abs(p['angle'] - 180) < 30]
            vertical_lines = [p for p in line_params if abs(p['angle'] - 90) < 30]
            
            # 合并每组内的相似直线
            merged_horizontal = self._merge_parallel_lines(horizontal_lines)
            merged_vertical = self._merge_parallel_lines(vertical_lines)
            
            # 返回合并后的直线
            merged_lines = []
            for line_data in merged_horizontal + merged_vertical:
                merged_lines.append([line_data['line']])
            
            return merged_lines
            
        except Exception as e:
            logger.error(f"直线合并失败: {e}")
            return lines  # 返回原始直线
    
    def _merge_parallel_lines(self, lines):
        """合并平行的直线"""
        if len(lines) <= 1:
            return lines
        
        # 按距离排序
        lines.sort(key=lambda x: x['distance'])
        
        merged = []
        used = set()
        
        for i, line1 in enumerate(lines):
            if i in used:
                continue
            
            # 找到与当前直线相似的所有直线
            similar_lines = [line1]
            used.add(i)
            
            for j, line2 in enumerate(lines):
                if j in used or j <= i:
                    continue
                
                # 检查是否为相似直线
                angle_diff = abs(line1['angle'] - line2['angle'])
                angle_diff = min(angle_diff, 180 - angle_diff)  # 处理角度环绕
                
                distance_diff = abs(line1['distance'] - line2['distance'])
                
                # 相似性判断：角度差小于15度，距离差小于20像素
                if angle_diff < 15 and distance_diff < 20:
                    similar_lines.append(line2)
                    used.add(j)
            
            # 合并相似直线：选择最长的线段
            best_line = max(similar_lines, key=lambda x: x['length'])
            merged.append(best_line)
        
        return merged
    
    def _black_rectangle_detection(self, gray):
        """黑色矩形检测备选方案"""
        try:
            # 高斯模糊
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 反向OTSU阈值（检测黑色区域）
            _, otsu_thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            # 轻微形态学操作
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            processed = cv2.morphologyEx(otsu_thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            processed = cv2.morphologyEx(processed, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # 寻找轮廓
            contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return {'success': False, 'reason': '未找到黑色区域轮廓'}
            
            # 筛选合适的轮廓
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if area < 100 or area > 10000:  # 面积筛选
                    continue
                
                # 计算最小外接矩形
                min_rect = cv2.minAreaRect(contour)
                rect_area = min_rect[1][0] * min_rect[1][1]
                
                if rect_area <= 0:
                    continue
                
                # 计算矩形度
                rectangularity = area / rect_area
                
                if rectangularity < 0.6:  # 矩形度筛选
                    continue
                
                # 获取旋转矩形的四个角点
                box_points = cv2.boxPoints(min_rect)
                box_points = np.int0(box_points)
                
                # 计算中心和边界框
                center = (int(min_rect[0][0]), int(min_rect[0][1]))
                
                x_coords = box_points[:, 0]
                y_coords = box_points[:, 1]
                x1, y1 = max(0, int(np.min(x_coords))), max(0, int(np.min(y_coords)))
                x2, y2 = min(gray.shape[1], int(np.max(x_coords))), min(gray.shape[0], int(np.max(y_coords)))
                
                return {
                    'success': True,
                    'center': center,
                    'bbox': [x1, y1, x2, y2],
                    'rectangle_points': box_points.tolist(),
                    'area': area,
                    'rectangularity': rectangularity,
                    'method': 'black_rectangle'
                }
            
            return {'success': False, 'reason': '未找到合适的黑色矩形'}
            
        except Exception as e:
            logger.error(f"黑色矩形检测失败: {e}")
            return {'success': False, 'reason': f'黑色矩形检测异常: {str(e)}'}
    
    def _analyze_merged_lines_to_rectangle(self, lines, image_shape):
        """分析合并后的直线构建矩形"""
        try:
            h, w = image_shape
            
            # 转换为直线方程
            line_equations = []
            line_angles = []
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # 计算角度
                angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
                if angle < 0:
                    angle += 180
                
                # 转换为直线方程
                if abs(x2 - x1) < 1e-6:  # 垂直线
                    a, b, c = 1, 0, -x1
                else:
                    slope = (y2 - y1) / (x2 - x1)
                    a = slope
                    b = -1
                    c = y1 - slope * x1
                
                # 归一化
                norm = np.sqrt(a*a + b*b)
                if norm > 0:
                    a, b, c = a/norm, b/norm, c/norm
                
                line_equations.append((a, b, c))
                line_angles.append(angle)
            
            # 按角度分组
            horizontal_lines = []
            vertical_lines = []
            
            for i, angle in enumerate(line_angles):
                if abs(angle) < 30 or abs(angle - 180) < 30:
                    horizontal_lines.append(i)
                elif abs(angle - 90) < 30:
                    vertical_lines.append(i)
            
            # 需要至少2条水平线和2条垂直线
            if len(horizontal_lines) < 2 or len(vertical_lines) < 2:
                return {'success': False, 'reason': f'直线分组不足 - 水平: {len(horizontal_lines)}, 垂直: {len(vertical_lines)}'}
            
            # 如果直线太多，选择最合适的
            if len(horizontal_lines) > 2:
                horizontal_lines = horizontal_lines[:2]  # 简单选择前2条
            if len(vertical_lines) > 2:
                vertical_lines = vertical_lines[:2]
            
            # 计算交点
            intersections = []
            selected_equations = []
            
            for idx in horizontal_lines + vertical_lines:
                selected_equations.append(line_equations[idx])
            
            for i in range(4):
                for j in range(i+1, 4):
                    intersection = self._line_intersection(selected_equations[i], selected_equations[j])
                    if intersection is not None:
                        x, y = intersection
                        if -w*0.2 <= x <= w*1.2 and -h*0.2 <= y <= h*1.2:  # 放宽边界
                            intersections.append((x, y))
            
            if len(intersections) < 4:
                return {'success': False, 'reason': f'交点数量不足: {len(intersections)}'}
            
            # 选择4个角点
            rectangle_points = self._select_rectangle_corners(intersections, w, h)
            
            if rectangle_points is None:
                return {'success': False, 'reason': '无法构成有效矩形'}
            
            # 计算矩形属性
            center_x = np.mean([p[0] for p in rectangle_points])
            center_y = np.mean([p[1] for p in rectangle_points])
            
            # 计算边界框
            x_coords = [p[0] for p in rectangle_points]
            y_coords = [p[1] for p in rectangle_points]
            x1, y1 = max(0, int(min(x_coords))), max(0, int(min(y_coords)))
            x2, y2 = min(w, int(max(x_coords))), min(h, int(max(y_coords)))
            
            area = abs((x2 - x1) * (y2 - y1))
            
            return {
                'success': True,
                'center': [int(center_x), int(center_y)],
                'bbox': [x1, y1, x2, y2],
                'rectangle_points': rectangle_points,
                'area': area,
                'line_count': len(lines),
                'intersection_count': len(intersections)
            }
            
        except Exception as e:
            logger.error(f"合并直线分析失败: {e}")
            return {'success': False, 'reason': f'合并直线分析异常: {str(e)}'}
    
    def _select_best_method(self, outer_quad_result, hough_result):
        """选择最佳检测方法"""
        # 优先使用外围四边形检测
        if outer_quad_result['success']:
            return outer_quad_result
        
        # 备选霍夫直线检测
        if hough_result['success']:
            return hough_result
        
        # 都失败则返回失败信息
        return {
            'success': False, 
            'reason': f"所有方法都失败 - 外围四边形: {outer_quad_result.get('reason', '未知')}, 霍夫: {hough_result.get('reason', '未知')}"
        }
    
    def _show_debug_window(self, roi_image, outer_quad_result, hough_result, best_result):
        """显示调试窗口 - 已禁用以测试性能"""
        # 性能测试模式：禁用所有图像显示
        pass

    def refine_detection_in_bbox(self, frame: np.ndarray, yolo_bbox: list, show_debug: bool = False) -> dict:
        """在YOLO检测框内进行精细化检测"""
        try:
            x1, y1, x2, y2 = map(int, yolo_bbox)
            
            # 确保边界框在图像范围内
            h, w = frame.shape[:2]
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(x1+1, min(x2, w))
            y2 = max(y1+1, min(y2, h))
            
            # 提取ROI区域
            roi_image = frame[y1:y2, x1:x2].copy()
            
            if roi_image.size == 0:
                return self._create_failed_result("提取的ROI图像为空")
            
            # 使用改进的检测方法
            roi_result = self.find_largest_rectangle_in_roi(roi_image, show_debug_windows=show_debug)
            
            if not roi_result['success']:
                return self._create_failed_result(roi_result.get('reason', '未知错误'))
            
            # 将ROI坐标转换回全局坐标
            roi_cx, roi_cy = roi_result['center']
            roi_x1, roi_y1, roi_x2, roi_y2 = roi_result['bbox']
            
            global_cx = x1 + roi_cx
            global_cy = y1 + roi_cy
            global_bbox = [
                x1 + roi_x1,
                y1 + roi_y1,
                x1 + roi_x2,
                y1 + roi_y2
            ]
            
            # 转换矩形角点到全局坐标
            global_rectangle_points = None
            if 'rectangle_points' in roi_result:
                global_rectangle_points = []
                for point in roi_result['rectangle_points']:
                    global_x = x1 + point[0]
                    global_y = y1 + point[1]
                    global_rectangle_points.append([global_x, global_y])
            
            debug_info = f"ROI: {roi_image.shape}, 方法: {roi_result['method']}"
            if 'merged_lines' in roi_result:
                debug_info += f", 合并直线: {roi_result['merged_lines']}"
            if 'rectangularity' in roi_result:
                debug_info += f", 矩形度: {roi_result['rectangularity']:.2f}"
            
            return {
                'success': True,
                'center': [global_cx, global_cy],
                'bbox': global_bbox,
                'rectangle_points': global_rectangle_points,
                'area': roi_result['area'],
                'method': roi_result['method'],
                'improvement': self._calculate_improvement(yolo_bbox, global_bbox),
                'debug_info': debug_info
            }
            
        except Exception as e:
            logger.error(f"精细化检测失败: {e}")
            return self._create_failed_result(f"异常: {str(e)}")

    def _create_failed_result(self, reason: str = "未知原因") -> dict:
        """创建失败结果"""
        return {
            'success': False,
            'center': None,
            'bbox': None,
            'area': 0,
            'improvement': 0,
            'debug_info': f"失败原因: {reason}"
        }
    
    def _calculate_improvement(self, yolo_bbox: list, refined_bbox: list) -> float:
        """计算改进程度 (面积变化比例)"""
        try:
            yolo_area = (yolo_bbox[2] - yolo_bbox[0]) * (yolo_bbox[3] - yolo_bbox[1])
            refined_area = (refined_bbox[2] - refined_bbox[0]) * (refined_bbox[3] - refined_bbox[1])
            
            if yolo_area > 0:
                improvement = abs(yolo_area - refined_area) / yolo_area * 100
                return round(improvement, 2)
            return 0
        except:
            return 0
    
    def _line_intersection(self, line1, line2):
        """计算两条直线的交点"""
        try:
            a1, b1, c1 = line1
            a2, b2, c2 = line2
            
            det = a1 * b2 - a2 * b1
            
            if abs(det) < 1e-10:
                return None
            
            x = (b1 * c2 - b2 * c1) / det
            y = (a2 * c1 - a1 * c2) / det
            
            return (x, y)
        except:
            return None
    
    def _select_rectangle_corners(self, intersections, width, height):
        """从交点中选择4个角点构成矩形"""
        try:
            if len(intersections) < 4:
                return None
            
            if len(intersections) > 4:
                points = np.array(intersections)
                hull = cv2.convexHull(points.astype(np.float32))
                hull_points = hull.reshape(-1, 2)
                
                if len(hull_points) >= 4:
                    if len(hull_points) > 4:
                        center_x = np.mean(hull_points[:, 0])
                        center_y = np.mean(hull_points[:, 1])
                        
                        distances = np.sqrt((hull_points[:, 0] - center_x)**2 + 
                                          (hull_points[:, 1] - center_y)**2)
                        
                        far_indices = np.argsort(distances)[-4:]
                        selected_points = hull_points[far_indices]
                    else:
                        selected_points = hull_points
                    
                    ordered_points = self._order_rectangle_points(selected_points)
                    return ordered_points
            
            elif len(intersections) == 4:
                points = np.array(intersections)
                ordered_points = self._order_rectangle_points(points)
                return ordered_points
            
            return None
            
        except Exception as e:
            logger.error(f"选择矩形角点失败: {e}")
            return None
    
    def _order_rectangle_points(self, points):
        """将4个点按照左上、右上、右下、左下的顺序排列"""
        try:
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            angles = np.arctan2(points[:, 1] - center_y, points[:, 0] - center_x)
            sorted_indices = np.argsort(angles)
            
            sorted_points = points[sorted_indices]
            
            # 找到最左上的点作为起始点
            top_left_idx = 0
            min_sum = float('inf')
            for i, point in enumerate(sorted_points):
                if point[0] + point[1] < min_sum:
                    min_sum = point[0] + point[1]
                    top_left_idx = i
            
            ordered_points = np.roll(sorted_points, -top_left_idx, axis=0)
            
            return ordered_points.tolist()
            
        except Exception as e:
            logger.error(f"排序矩形点失败: {e}")
            return points.tolist()

def create_refinement_processor(config: dict = None) -> SquareRefinement:
    """创建精细化处理器的工厂函数"""
    default_config = {
        'min_contour_area': 100,
        'max_contour_area': 10000,
        'blur_kernel_size': 5,
        'morph_kernel_size': 3
    }
    
    if config:
        default_config.update(config)
    
    return SquareRefinement(**default_config)

def visualize_comparison(frame: np.ndarray, 
                        yolo_result: dict, 
                        refinement_result: dict) -> np.ndarray:
    """对比可视化YOLO检测和OpenCV精细化结果"""
    vis_frame = frame.copy()
    
    try:
        if yolo_result['found']:
            # 绘制YOLO检测框 (黄色)
            x1, y1, x2, y2 = yolo_result['bbox']
            cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            
            # 绘制YOLO中心点 (蓝色圆圈)
            yolo_cx, yolo_cy = yolo_result['center']
            cv2.circle(vis_frame, (yolo_cx, yolo_cy), 8, (255, 0, 0), 2)
            cv2.circle(vis_frame, (yolo_cx, yolo_cy), 3, (255, 0, 0), -1)
            
            # YOLO标签
            yolo_label = f"YOLO: {yolo_result['confidence']:.2f}"
            cv2.putText(vis_frame, yolo_label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            if refinement_result['success']:
                # 绘制OpenCV重构的四边形 (绿色)
                if 'rectangle_points' in refinement_result and refinement_result['rectangle_points'] is not None:
                    points = refinement_result['rectangle_points']
                    points_np = np.array(points, dtype=np.int32)
                    cv2.polylines(vis_frame, [points_np], True, (0, 255, 0), 3)
                    
                    # 绘制角点编号
                    for i, point in enumerate(points):
                        x, y = int(point[0]), int(point[1])
                        cv2.circle(vis_frame, (x, y), 5, (0, 255, 0), -1)
                        cv2.putText(vis_frame, str(i), (x+8, y-8), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                else:
                    # 如果没有角点信息，绘制普通矩形
                    rx1, ry1, rx2, ry2 = refinement_result['bbox']
                    cv2.rectangle(vis_frame, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)
                
                # 绘制OpenCV中心点 (红色十字)
                cv_cx, cv_cy = refinement_result['center']
                cv2.line(vis_frame, (cv_cx-15, cv_cy), (cv_cx+15, cv_cy), (0, 0, 255), 3)
                cv2.line(vis_frame, (cv_cx, cv_cy-15), (cv_cx, cv_cy+15), (0, 0, 255), 3)
                cv2.circle(vis_frame, (cv_cx, cv_cy), 6, (0, 0, 255), -1)
                
                # OpenCV标签，显示检测方法
                method = refinement_result.get('method', 'unknown')
                cv_label = f"Method: {method}"
                cv2.putText(vis_frame, cv_label, (x1, y1-30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 连接两个中心点
                cv2.line(vis_frame, (yolo_cx, yolo_cy), (cv_cx, cv_cy), (255, 255, 255), 2)
                
                # 计算中心点偏移距离
                distance = np.sqrt((cv_cx - yolo_cx)**2 + (cv_cy - yolo_cy)**2)
                dist_text = f"Offset: {distance:.1f}px"
                cv2.putText(vis_frame, dist_text, (10, vis_frame.shape[0]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                # OpenCV处理失败的提示
                fail_reason = refinement_result.get('debug_info', 'Unknown error')
                fail_text = f"OpenCV: {fail_reason}"
                cv2.putText(vis_frame, fail_text, (x1, y1-30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # 添加图例
        legend_y = 30
        cv2.putText(vis_frame, "YOLO (Yellow Box + Blue Circle)", (10, legend_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(vis_frame, "OpenCV (Green Quadrilateral + Red Cross)", (10, legend_y + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
    except Exception as e:
        logger.error(f"对比可视化失败: {e}")
    
    return vis_frame

if __name__ == "__main__":
    # OpenCV外围四边形检测测试 - 性能测试模式
    import sys
    import time
    sys.path.append(r"./")
    from YOLO11_Detect_YUV420SP import VisionDetector

    print("启动YOLO vs 外围四边形检测性能测试...")
    print("=" * 50)
    print("性能测试模式：已禁用所有图像显示")
    print("=" * 50)

    try:
        # 初始化检测器和处理器
        detector = VisionDetector("/root/square/detect/1.0.bin")
        processor = create_refinement_processor()
        
        # 摄像头设置
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 性能统计
        frame_count = 0
        fps_start_time = time.time()
        yolo_success_count = 0
        opencv_success_count = 0
        outer_quad_success_count = 0
        hough_success_count = 0
        total_offset = 0
        offset_count = 0
        
        # 计时器
        total_yolo_time = 0
        total_opencv_time = 0
        
        print("\n按键说明:")
        print("'q' - 退出")
        print("'s' - 显示统计信息")
        print("'r' - 重置统计")
        print("\n开始性能测试...")
        
        # 性能测试模式：禁用所有图像显示
        show_opencv_debug = False
        frame_counter = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头帧")
                break
            
            frame_count += 1
            frame_counter += 1
            
            # YOLO检测计时
            yolo_start = time.time()
            yolo_result = detector.get_best_detection(frame)
            yolo_time = time.time() - yolo_start
            total_yolo_time += yolo_time
            
            # 初始化精细化结果
            refinement_result = {'success': False}
            opencv_time = 0
            
            if yolo_result['found']:
                yolo_success_count += 1
                
                # OpenCV精细化处理计时
                opencv_start = time.time()
                refinement_result = processor.refine_detection_in_bbox(
                    frame, yolo_result['bbox'], show_debug=False)  # 强制关闭调试
                opencv_time = time.time() - opencv_start
                total_opencv_time += opencv_time
                
                if refinement_result['success']:
                    opencv_success_count += 1
                    
                    # 统计方法使用情况
                    method = refinement_result.get('method', 'unknown')
                    if 'outer_quad' in method:
                        outer_quad_success_count += 1
                    elif 'hough' in method:
                        hough_success_count += 1
                    
                    # 计算中心点偏移
                    yolo_cx, yolo_cy = yolo_result['center']
                    cv_cx, cv_cy = refinement_result['center']
                    offset = np.sqrt((cv_cx - yolo_cx)**2 + (cv_cy - yolo_cy)**2)
                    total_offset += offset
                    offset_count += 1
            
            # 性能测试模式：不进行图像显示
            # vis_frame = visualize_comparison(frame, yolo_result, refinement_result)
            # cv2.imshow("YOLO vs Outer Quadrilateral Detection", vis_frame)
            
            # 每30帧显示一次性能统计
            if frame_count % 30 == 0:
                elapsed = time.time() - fps_start_time
                current_fps = 30 / elapsed
                avg_yolo_ms = (total_yolo_time / frame_count) * 1000
                avg_opencv_ms = (total_opencv_time / max(yolo_success_count, 1)) * 1000
                
                print(f"\n当前FPS: {current_fps:.1f}")
                print(f"YOLO平均耗时: {avg_yolo_ms:.2f}ms")
                print(f"OpenCV平均耗时: {avg_opencv_ms:.2f}ms")
                print(f"YOLO成功率: {yolo_success_count/frame_count*100:.1f}%")
                if yolo_success_count > 0:
                    print(f"OpenCV成功率: {opencv_success_count/yolo_success_count*100:.1f}%")
                
                fps_start_time = time.time()
            
            # 实时结果显示（每5帧输出一次）
            if frame_count % 5 == 0:
                if yolo_result['found']:
                    if refinement_result['success']:
                        yolo_center = yolo_result['center']
                        cv_center = refinement_result['center']
                        offset = np.sqrt((cv_center[0] - yolo_center[0])**2 + (cv_center[1] - yolo_center[1])**2)
                        method = refinement_result.get('method', 'unknown')
                        print(f"\r帧{frame_count} | YOLO: {yolo_center} | {method}: {cv_center} | 偏移: {offset:.1f}px | YOLO: {yolo_time*1000:.1f}ms, OpenCV: {opencv_time*1000:.1f}ms", end="", flush=True)
                    else:
                        debug_info = refinement_result.get('debug_info', '未知错误')
                        print(f"\r帧{frame_count} | YOLO: {yolo_result['center']} | 四边形: 失败 | YOLO: {yolo_time*1000:.1f}ms", end="", flush=True)
                else:
                    print(f"\r帧{frame_count} | 未检测到方块 | YOLO: {yolo_time*1000:.1f}ms", end="", flush=True)
            
            # 按键处理（非阻塞）
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                print("\n" + "="*80)
                print("性能测试统计信息:")
                print(f"总帧数: {frame_count}")
                print(f"平均FPS: {frame_count/(time.time() - (fps_start_time if frame_count <= 30 else fps_start_time - 30/current_fps)):.2f}")
                print(f"YOLO成功率: {yolo_success_count/frame_count*100:.1f}% ({yolo_success_count}/{frame_count})")
                if yolo_success_count > 0:
                    print(f"OpenCV总成功率: {opencv_success_count/yolo_success_count*100:.1f}% ({opencv_success_count}/{yolo_success_count})")
                    print(f"外围四边形成功: {outer_quad_success_count} 次")
                    print(f"霍夫直线成功: {hough_success_count} 次")
                print(f"YOLO平均耗时: {(total_yolo_time/frame_count)*1000:.2f}ms")
                if yolo_success_count > 0:
                    print(f"OpenCV平均耗时: {(total_opencv_time/yolo_success_count)*1000:.2f}ms")
                if offset_count > 0:
                    print(f"平均中心点偏移: {total_offset/offset_count:.1f} 像素")
                print("="*80)
            elif key == ord('r'):
                frame_count = 0
                yolo_success_count = 0
                opencv_success_count = 0
                outer_quad_success_count = 0
                hough_success_count = 0
                total_offset = 0
                offset_count = 0
                total_yolo_time = 0
                total_opencv_time = 0
                fps_start_time = time.time()
                print("\n统计信息已重置")
    
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'cap' in locals():
                cap.release()
            # cv2.destroyAllWindows()  # 不需要销毁窗口，因为没有创建
            
            if frame_count > 0:
                total_time = time.time() - (fps_start_time if frame_count <= 30 else fps_start_time - 30/30)
                print("\n" + "="*80)
                print("最终性能测试结果:")
                print(f"总帧数: {frame_count}")
                print(f"总运行时间: {total_time:.2f}秒")
                print(f"平均FPS: {frame_count/total_time:.2f}")
                print(f"YOLO成功率: {yolo_success_count/frame_count*100:.1f}%")
                if yolo_success_count > 0:
                    print(f"OpenCV总成功率: {opencv_success_count/yolo_success_count*100:.1f}%")
                    print(f"外围四边形检测: {outer_quad_success_count} 次成功")
                    print(f"霍夫直线检测: {hough_success_count} 次成功")
                print(f"YOLO平均耗时: {(total_yolo_time/frame_count)*1000:.2f}ms")
                if yolo_success_count > 0:
                    print(f"OpenCV平均耗时: {(total_opencv_time/yolo_success_count)*1000:.2f}ms")
                    print(f"总处理耗时: {((total_yolo_time + total_opencv_time)/frame_count)*1000:.2f}ms")
                if offset_count > 0:
                    print(f"平均中心点偏移: {total_offset/offset_count:.1f} 像素")
                print("="*80)
            
            print("性能测试完成")
        except:
            pass
