#!/user/bin/env python

# Copyright (c) 2024，WuChao D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import numpy as np
from scipy.special import softmax
from hobot_dnn import pyeasy_dnn as dnn
import time  # 确保正确导入time模块
import logging
import threading
import queue

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("YOLO_Vision")

class BaseModel:
    def __init__(self, model_file: str) -> None:
        try:
            self.quantize_model = dnn.load(model_file)
            logger.info(f"模型加载成功: {model_file}")
        except Exception as e:
            logger.error(f"模型加载失败: {e}")
            raise e

        self.model_input_height, self.model_input_weight = self.quantize_model[0].inputs[0].properties.shape[2:4]

    def resizer(self, img: np.ndarray) -> np.ndarray:
        img_h, img_w = img.shape[0:2]
        self.y_scale, self.x_scale = img_h/self.model_input_height, img_w/self.model_input_weight
        return cv2.resize(img, (self.model_input_height, self.model_input_weight), interpolation=cv2.INTER_NEAREST)
    
    def bgr2nv12(self, bgr_img: np.ndarray) -> np.ndarray:
        bgr_img = self.resizer(bgr_img)
        height, width = bgr_img.shape[0], bgr_img.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.zeros_like(yuv420p)
        nv12[:height * width] = y
        nv12[height * width:] = uv_packed
        return nv12

    def forward(self, input_tensor: np.array) -> list[dnn.pyDNNTensor]:
        return self.quantize_model[0].forward(input_tensor)

    def c2numpy(self, outputs) -> list[np.array]:
        return [dnnTensor.buffer for dnnTensor in outputs]

class YOLO11_Detect(BaseModel):
    def __init__(self, model_file: str, conf: float = 0.25, iou: float = 0.45):
        super().__init__(model_file)
        
        # 初始化量化系数
        self.s_bboxes_scale = self.quantize_model[0].outputs[0].properties.scale_data[np.newaxis, :]
        self.m_bboxes_scale = self.quantize_model[0].outputs[1].properties.scale_data[np.newaxis, :]
        self.l_bboxes_scale = self.quantize_model[0].outputs[2].properties.scale_data[np.newaxis, :]

        # DFL权重
        self.weights_static = np.array([i for i in range(16)]).astype(np.float32)[np.newaxis, np.newaxis, :]

        # 锚点
        self.s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                            np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
        self.m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                            np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
        self.l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                            np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)

        self.conf = conf
        self.iou = iou
        self.conf_inverse = -np.log(1/conf - 1)

    def postProcess(self, outputs: list[np.ndarray]) -> tuple[list]:
        s_bboxes = outputs[0].reshape(-1, 64)
        m_bboxes = outputs[1].reshape(-1, 64)
        l_bboxes = outputs[2].reshape(-1, 64)
        s_clses = outputs[3].reshape(-1, 1)
        m_clses = outputs[4].reshape(-1, 1)
        l_clses = outputs[5].reshape(-1, 1)

        # 分类筛选
        s_max_scores = np.max(s_clses, axis=1)
        s_valid_indices = np.flatnonzero(s_max_scores >= self.conf_inverse)
        s_ids = np.argmax(s_clses[s_valid_indices, : ], axis=1)
        s_scores = s_max_scores[s_valid_indices]

        m_max_scores = np.max(m_clses, axis=1)
        m_valid_indices = np.flatnonzero(m_max_scores >= self.conf_inverse)
        m_ids = np.argmax(m_clses[m_valid_indices, : ], axis=1)
        m_scores = m_max_scores[m_valid_indices]

        l_max_scores = np.max(l_clses, axis=1)
        l_valid_indices = np.flatnonzero(l_max_scores >= self.conf_inverse)
        l_ids = np.argmax(l_clses[l_valid_indices, : ], axis=1)
        l_scores = l_max_scores[l_valid_indices]

        # Sigmoid计算
        s_scores = 1 / (1 + np.exp(-s_scores))
        m_scores = 1 / (1 + np.exp(-m_scores))
        l_scores = 1 / (1 + np.exp(-l_scores))

        # 边界框处理
        s_bboxes_float32 = s_bboxes[s_valid_indices,:]
        m_bboxes_float32 = m_bboxes[m_valid_indices,:]
        l_bboxes_float32 = l_bboxes[l_valid_indices,:]

        # 转换为边界框坐标
        s_ltrb_indices = np.sum(softmax(s_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        s_anchor_indices = self.s_anchor[s_valid_indices, :]
        s_x1y1 = s_anchor_indices - s_ltrb_indices[:, 0:2]
        s_x2y2 = s_anchor_indices + s_ltrb_indices[:, 2:4]
        s_dbboxes = np.hstack([s_x1y1, s_x2y2])*8

        m_ltrb_indices = np.sum(softmax(m_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        m_anchor_indices = self.m_anchor[m_valid_indices, :]
        m_x1y1 = m_anchor_indices - m_ltrb_indices[:, 0:2]
        m_x2y2 = m_anchor_indices + m_ltrb_indices[:, 2:4]
        m_dbboxes = np.hstack([m_x1y1, m_x2y2])*16

        l_ltrb_indices = np.sum(softmax(l_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        l_anchor_indices = self.l_anchor[l_valid_indices,:]
        l_x1y1 = l_anchor_indices - l_ltrb_indices[:, 0:2]
        l_x2y2 = l_anchor_indices + l_ltrb_indices[:, 2:4]
        l_dbboxes = np.hstack([l_x1y1, l_x2y2])*32

        # 合并结果
        dbboxes = np.concatenate((s_dbboxes, m_dbboxes, l_dbboxes), axis=0)
        scores = np.concatenate((s_scores, m_scores, l_scores), axis=0)
        ids = np.concatenate((s_ids, m_ids, l_ids), axis=0)

        # NMS
        indices = cv2.dnn.NMSBoxes(dbboxes, scores, self.conf, self.iou)

        if len(indices) > 0:
            # 还原到原始图像尺度
            bboxes = dbboxes[indices] * np.array([self.x_scale, self.y_scale, self.x_scale, self.y_scale])
            bboxes = bboxes.astype(np.int32)
            return ids[indices], scores[indices], bboxes
        else:
            return [], [], []

class VisionDetector:
    """视觉检测模块 - 对外接口，优化实时性能"""
    
    def __init__(self, model_path: str, conf_thresh: float = 0.25, iou_thresh: float = 0.45):
        """
        初始化视觉检测器
        :param model_path: 模型文件路径
        :param conf_thresh: 置信度阈值
        :param iou_thresh: IoU阈值
        """
        self.model = YOLO11_Detect(model_path, conf_thresh, iou_thresh)
        self.class_names = ["square"]
        
        # 简化性能优化参数
        self.frame_count = 0
        self.last_detection_result = {'found': False, 'center': None, 'bbox': None}
        
        logger.info("视觉检测器初始化完成")
    
    def detect_fast(self, frame: np.ndarray) -> dict:
        """
        快速检测模式 - 简化版本
        :param frame: 输入图像 (BGR格式)
        :return: 检测结果字典
        """
        self.frame_count += 1
        
        # 简化跳帧：每2帧检测1次
        if self.frame_count % 2 == 0:
            try:
                result = self.detect(frame)
                if result['success'] and result['count'] > 0:
                    self.last_detection_result = self.get_best_detection_from_result(result)
            except Exception as e:
                logger.error(f"检测失败: {e}")
        
        return self.last_detection_result

    def detect(self, frame: np.ndarray) -> dict:
        """
        检测图像中的目标 - 优化版本
        :param frame: 输入图像 (BGR格式)
        :return: 检测结果字典
        """
        try:
            # 添加输入验证
            if frame is None or frame.size == 0:
                logger.warning("输入图像为空")
                return {'success': False, 'detections': [], 'count': 0}
            
            # 预处理 - 添加异常处理
            try:
                input_tensor = self.model.bgr2nv12(frame)
            except Exception as e:
                logger.error(f"图像预处理失败: {e}")
                return {'success': False, 'detections': [], 'count': 0}
            
            # 推理 - 添加超时和异常处理
            try:
                outputs = self.model.c2numpy(self.model.forward(input_tensor))
            except Exception as e:
                logger.error(f"模型推理失败: {e}")
                return {'success': False, 'detections': [], 'count': 0}
            
            # 后处理 - 添加异常处理
            try:
                ids, scores, bboxes = self.model.postProcess(outputs)
            except Exception as e:
                logger.error(f"后处理失败: {e}")
                return {'success': False, 'detections': [], 'count': 0}
            
            # 构建结果
            detections = []
            for class_id, score, bbox in zip(ids, scores, bboxes):
                try:
                    x1, y1, x2, y2 = bbox
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    width = x2 - x1
                    height = y2 - y1
                    
                    detection = {
                        'class_id': int(class_id),
                        'class_name': self.class_names[class_id],
                        'confidence': float(score),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [int(center_x), int(center_y)],
                        'size': [int(width), int(height)]
                    }
                    detections.append(detection)
                except Exception as e:
                    logger.warning(f"处理单个检测结果失败: {e}")
                    continue
            
            result = {
                'success': True,
                'detections': detections,
                'count': len(detections)
            }
            
            return result
            
        except Exception as e:
            logger.error(f"检测失败: {e}")
            return {
                'success': False,
                'detections': [],
                'count': 0,
                'error': str(e)
            }
    
    def get_best_detection_from_result(self, result: dict, target_class: str = "square") -> dict:
        """
        从检测结果中获取最佳目标
        :param result: 检测结果
        :param target_class: 目标类别
        :return: 最佳检测结果
        """
        if not result['success'] or result['count'] == 0:
            return {'found': False, 'center': None, 'bbox': None}
        
        # 筛选目标类别
        target_detections = [d for d in result['detections'] if d['class_name'] == target_class]
        
        if not target_detections:
            return {'found': False, 'center': None, 'bbox': None}
        
        # 找到置信度最高的检测结果
        best_detection = max(target_detections, key=lambda x: x['confidence'])
        
        return {
            'found': True,
            'center': best_detection['center'],
            'bbox': best_detection['bbox'],
            'confidence': best_detection['confidence'],
            'size': best_detection['size']
        }
    
    def get_best_detection(self, frame: np.ndarray, target_class: str = "square") -> dict:
        """
        获取最佳检测结果（置信度最高的目标）- 简化版本
        :param frame: 输入图像
        :param target_class: 目标类别名称
        :return: 最佳检测结果
        """
        # 直接进行检测，不使用复杂的缓存机制
        try:
            result = self.detect(frame)
            return self.get_best_detection_from_result(result, target_class)
        except Exception as e:
            logger.error(f"获取最佳检测失败: {e}")
            return {'found': False, 'center': None, 'bbox': None}

    def draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        """
        在图像上绘制检测结果 - 优化版本
        :param frame: 输入图像
        :param detections: 检测结果列表
        :return: 绘制后的图像
        """
        if frame is None or len(detections) == 0:
            return frame
        
        result_frame = frame.copy()
        
        try:
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                center_x, center_y = detection['center']
                confidence = detection['confidence']
                class_name = detection['class_name']
                
                # 绘制边界框
                cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 绘制中心点
                cv2.circle(result_frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 绘制标签
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(result_frame, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        except Exception as e:
            logger.error(f"绘制检测结果失败: {e}")
            return frame
        
        return result_frame
    
    def draw_best_detection(self, frame: np.ndarray, detection_result: dict) -> np.ndarray:
        """
        绘制最佳检测结果
        :param frame: 输入图像
        :param detection_result: get_best_detection的返回结果
        :return: 绘制后的图像
        """
        if not detection_result['found']:
            return frame
        
        try:
            result_frame = frame.copy()
            center_x, center_y = detection_result['center']
            x1, y1, x2, y2 = detection_result['bbox']
            confidence = detection_result['confidence']
            
            # 绘制边界框
            cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(result_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 绘制十字线
            cv2.line(result_frame, (center_x-10, center_y), (center_x+10, center_y), (0, 0, 255), 2)
            cv2.line(result_frame, (center_x, center_y-10), (center_x, center_y+10), (0, 0, 255), 2)
            
            # 绘制标签
            label = f"square: {confidence:.2f}"
            cv2.putText(result_frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            return result_frame
        except Exception as e:
            logger.error(f"绘制最佳检测结果失败: {e}")
            return frame
    
    def reset_detection_cache(self):
        """重置检测缓存"""
        self.last_detection_result = {'found': False, 'center': None, 'bbox': None}
        self.frame_count = 0

class AsyncVisionDetector(VisionDetector):
    """异步视觉检测器 - 进一步优化性能"""
    
    def __init__(self, model_path: str, conf_thresh: float = 0.25, iou_thresh: float = 0.45):
        super().__init__(model_path, conf_thresh, iou_thresh)
        
        # 异步检测相关
        self.detection_queue = queue.Queue(maxsize=2)  # 限制队列大小
        self.result_queue = queue.Queue(maxsize=5)
        self.detection_thread = None
        self.async_running = False
        
    def start_async_detection(self):
        """启动异步检测线程"""
        if self.async_running:
            return
        
        self.async_running = True
        self.detection_thread = threading.Thread(target=self._detection_worker, daemon=True)
        self.detection_thread.start()
        logger.info("异步检测线程已启动")
    
    def stop_async_detection(self):
        """停止异步检测线程"""
        self.async_running = False
        if self.detection_thread:
            self.detection_thread.join(timeout=1.0)
        logger.info("异步检测线程已停止")
    
    def _detection_worker(self):
        """检测工作线程"""
        while self.async_running:
            try:
                # 获取待检测的图像
                frame = self.detection_queue.get(timeout=0.1)
                
                # 执行检测
                result = super().detect(frame)
                
                # 将结果放入结果队列
                if not self.result_queue.full():
                    self.result_queue.put(result)
                
                self.detection_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"异步检测工作线程错误: {e}")
    
    def async_detect(self, frame: np.ndarray) -> dict:
        """
        异步检测
        :param frame: 输入图像
        :return: 检测结果
        """
        # 将图像放入检测队列
        if not self.detection_queue.full():
            try:
                self.detection_queue.put_nowait(frame.copy())
            except queue.Full:
                pass
        
        # 尝试获取最新的检测结果
        try:
            result = self.result_queue.get_nowait()
            if result['success'] and result['count'] > 0:
                self.last_detection_result = self.get_best_detection_from_result(result)
            return self.last_detection_result
        except queue.Empty:
            return self.last_detection_result

if __name__ == "__main__":
    # 测试代码 - 性能测试模式（无图像显示）
    print("启动YOLO11检测性能测试...")
    print("性能测试模式：已禁用所有图像显示")
    
    try:
        detector = VisionDetector("/root/square/detect/1.0.bin")
        
        # 测试摄像头
        cap = cv2.VideoCapture(0)
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 性能监控
        fps_counter = 0
        fps_start_time = time.time()
        total_detection_time = 0
        successful_detections = 0
        
        print("摄像头已启动，按 'q' 退出")
        print("按 'r' 重置检测缓存")
        print("按 's' 显示详细统计")
        
        while True:
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头帧")
                break
            
            # 直接进行检测，不复杂的跳帧
            try:
                detection_start = time.time()
                detection_result = detector.get_best_detection(frame)
                detection_time = time.time() - detection_start
                total_detection_time += detection_time
                
                # 性能测试模式：不进行图像显示
                # frame = detector.draw_best_detection(frame, detection_result)
                
                if detection_result['found']:
                    successful_detections += 1
                    center_x, center_y = detection_result['center']
                    confidence = detection_result['confidence']
                    if fps_counter % 10 == 0:  # 每10帧输出一次，减少输出频率
                        print(f"\r帧{fps_counter}: 检测到方块 ({center_x}, {center_y}) 置信度:{confidence:.2f} 耗时:{detection_time*1000:.1f}ms", end="", flush=True)
                else:
                    if fps_counter % 10 == 0:
                        print(f"\r帧{fps_counter}: 未检测到方块 耗时:{detection_time*1000:.1f}ms", end="", flush=True)
                
            except Exception as e:
                logger.error(f"检测过程出错: {e}")
                continue
            
            # 性能测试模式：不显示图像
            # cv2.imshow('YOLO Detection', frame)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                detector.reset_detection_cache()
                total_detection_time = 0
                successful_detections = 0
                fps_counter = 0
                fps_start_time = time.time()
                print("\n检测缓存和统计已重置")
            elif key == ord('s'):
                if fps_counter > 0:
                    elapsed = time.time() - fps_start_time
                    avg_fps = fps_counter / elapsed
                    avg_detection_time = (total_detection_time / fps_counter) * 1000
                    success_rate = (successful_detections / fps_counter) * 100
                    
                    print(f"\n" + "="*60)
                    print(f"性能统计 (基于{fps_counter}帧):")
                    print(f"平均FPS: {avg_fps:.2f}")
                    print(f"平均检测耗时: {avg_detection_time:.2f}ms")
                    print(f"检测成功率: {success_rate:.1f}%")
                    print(f"成功检测: {successful_detections} / {fps_counter}")
                    print("="*60)
            
            # FPS统计（每30帧输出一次详细信息）
            fps_counter += 1
            if fps_counter % 30 == 0:
                fps_elapsed = time.time() - fps_start_time
                if fps_elapsed > 0:
                    avg_fps = 30 / fps_elapsed
                    avg_detection_ms = (total_detection_time / fps_counter) * 1000
                    print(f"\n当前FPS: {avg_fps:.1f}, 平均检测耗时: {avg_detection_ms:.2f}ms")
                fps_start_time = time.time()
    
    except Exception as e:
        logger.error(f"测试程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'cap' in locals():
                cap.release()
            # cv2.destroyAllWindows()  # 性能测试模式不需要
            
            # 最终统计
            if 'fps_counter' in locals() and fps_counter > 0:
                print(f"\n" + "="*60)
                print("最终性能测试结果:")
                print(f"总处理帧数: {fps_counter}")
                print(f"平均检测耗时: {(total_detection_time/fps_counter)*1000:.2f}ms")
                print(f"检测成功率: {(successful_detections/fps_counter)*100:.1f}%")
                print(f"成功检测帧数: {successful_detections}")
                print("="*60)
            
            print("性能测试完成")
        except:
            pass