#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import torch
import rospy
import threading
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov8_ros_msgs.msg import YoloDetection
from std_msgs.msg import String
import numpy as np

filter = 2


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


class CameraState:
    """Per-camera copy of the original stability state."""
    def __init__(self):
        self.last_type = None
        self.stable_count = 0
        self.switch_pending = None
        self.no_type_count = 0
        self.frame_seq = 0
        self.last_image_pub_time = rospy.Time(0)

    def reset(self):
        self.last_type = None
        self.stable_count = 0
        self.switch_pending = None
        self.no_type_count = 0


class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)

        # ===== 原有参数，名字和默认值保持兼容 =====
        weight_path = rospy.get_param('~weight_path', '/home/abot/kk_ws/src/Yolov8_ros/yolov8_ros/weights/mix.pt')
        conf = rospy.get_param('~conf', 0.5)
        use_cpu = rospy.get_param('~use_cpu', False)
        self.enhanced = rospy.get_param('~enhanced', True)
        self.visualize = _as_bool(rospy.get_param('~visualize', False))

        self.device = 'cpu' if use_cpu else ('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(weight_path)
        if torch.cuda.is_available() and not use_cpu:
            self.model.to(self.device)
        self.model.conf = conf
        self.conf = conf

        self.bridge = CvBridge()
        self.model_lock = threading.Lock()

        # ===== 原有输入参数 =====
        self.image_topic_down = rospy.get_param('~image_topic_down', '/down/usb_cam/image_raw')
        self.image_topic_forward = rospy.get_param('~image_topic_forward', '/forward/usb_cam_front/image_raw')
        self.image_topic = rospy.get_param('~image_topic', '')

        # ===== 原有输出参数 =====
        pub_topic = rospy.get_param('~pub_topic', '/yolo/detections')
        image_pub_topic = rospy.get_param('~image_pub_topic', '/yolov8_detection/image_result')

        # ===== 新增输出参数：不影响原有 pub_topic，额外给前/下识别结果各自一个话题 =====
        # 主程序原来订阅 /yolo/detections 的话，仍然只收到 active_camera 当前视角的识别结果。
        # 需要同时看两路识别时，订阅下面两个话题。
        pub_topic_down = rospy.get_param('~pub_topic_down', '/yolo/detections_down')
        pub_topic_forward = rospy.get_param('~pub_topic_forward', '/yolo/detections_forward')

        self.position_pub = rospy.Publisher(pub_topic, YoloDetection, queue_size=1)
        self.position_pub_down = rospy.Publisher(pub_topic_down, YoloDetection, queue_size=1)
        self.position_pub_forward = rospy.Publisher(pub_topic_forward, YoloDetection, queue_size=1)
        self.image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=1)

        # ===== 运行模式 =====
        self.single_camera = _as_bool(rospy.get_param('~single_camera', False))
        # 兼容 ros_predict.launch / camera_detect_front.launch 这种单 image_topic 调用方式
        if self.image_topic and not rospy.has_param('~image_topic_down') and not rospy.has_param('~image_topic_forward'):
            self.single_camera = True

        active_camera = str(rospy.get_param('~active_camera', 'down')).strip().lower()
        self.active_camera = active_camera if active_camera in ('down', 'forward') else 'down'

        # dual 模式下默认两路都识别；如需恢复老逻辑，可 launch 里设 recognize_mode:=active
        self.recognize_mode = str(rospy.get_param('~recognize_mode', 'both')).strip().lower()
        if self.recognize_mode not in ('both', 'active'):
            self.recognize_mode = 'both'

        # 只发布前置摄像头带框图片，满足评委只看前方框的要求
        self.publish_front_image_only = _as_bool(rospy.get_param('~publish_front_image_only', True))
        self.publish_image = _as_bool(rospy.get_param('~publish_image', True))
        self.image_pub_fps = float(rospy.get_param('~image_pub_fps', 0.0))  # 0=不限速，保持兼容
        self.buff_size = int(rospy.get_param('~buff_size', 5242880))

        # 可选降载参数；默认 1，不改变识别帧输入逻辑
        self.frame_skip_down = max(1, int(rospy.get_param('~frame_skip_down', rospy.get_param('~frame_skip', 1))))
        self.frame_skip_forward = max(1, int(rospy.get_param('~frame_skip_forward', rospy.get_param('~frame_skip', 1))))

        self.states = {
            'down': CameraState(),
            'forward': CameraState(),
        }

        if self.single_camera:
            if not self.image_topic:
                self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
            # 单摄时根据 active_camera 判断类别过滤；front launch 会传 active_camera=forward
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback_single, queue_size=1, buff_size=self.buff_size)
            rospy.loginfo('单摄模式订阅: %s, active_camera=%s', self.image_topic, self.active_camera)
        else:
            # 双路全部订阅，默认双路全部识别
            self.sub_down = rospy.Subscriber(self.image_topic_down, Image, self.image_callback_down, queue_size=1, buff_size=self.buff_size)
            self.sub_forward = rospy.Subscriber(self.image_topic_forward, Image, self.image_callback_forward, queue_size=1, buff_size=self.buff_size)
            self.switch_sub = rospy.Subscriber('/yolo/switch_camera', String, self.switch_callback, queue_size=1)
            rospy.loginfo('双摄模式订阅: down=%s, forward=%s, recognize_mode=%s, active_camera=%s',
                          self.image_topic_down, self.image_topic_forward, self.recognize_mode, self.active_camera)

        rospy.loginfo('加载模型: %s', weight_path)
        rospy.loginfo('使用设备: %s', self.device)
        rospy.loginfo('active识别结果: %s', pub_topic)
        rospy.loginfo('下方识别结果: %s', pub_topic_down)
        rospy.loginfo('前方识别结果: %s', pub_topic_forward)
        rospy.loginfo('前方带框图像: %s', image_pub_topic)
        rospy.loginfo('开始实时检测，按Ctrl+C退出...')

    # ================== 回调入口 ==================
    def image_callback_single(self, msg):
        self.image_callback(msg, self.active_camera)

    def image_callback_down(self, msg):
        if self.recognize_mode == 'active' and self.active_camera != 'down':
            return
        self.image_callback(msg, 'down')

    def image_callback_forward(self, msg):
        if self.recognize_mode == 'active' and self.active_camera != 'forward':
            return
        self.image_callback(msg, 'forward')

    def switch_callback(self, msg):
        target_cam = msg.data.strip().lower()
        if target_cam not in ('down', 'forward'):
            rospy.logwarn('未知摄像头切换指令: %s', msg.data)
            return
        if target_cam == self.active_camera:
            return

        rospy.loginfo('收到主程序指令，active检测结果切换到 %s 摄像头', target_cam)
        self.active_camera = target_cam
        # 不清空两路各自状态，避免 both 模式下另一摄像头稳定识别被切换指令打断。

    # ================== 原识别逻辑：按摄像头角色独立执行 ==================
    def image_callback(self, msg, camera_role):
        state = self.states[camera_role]
        state.frame_seq += 1
        if camera_role == 'down' and state.frame_seq % self.frame_skip_down != 0:
            return
        if camera_role == 'forward' and state.frame_seq % self.frame_skip_forward != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr('图像转换失败(%s): %s', camera_role, e)
            return

        # YOLO 模型对象串行调用，避免 rospy 双订阅回调并发推理造成卡死/显存异常。
        try:
            with self.model_lock:
                with torch.no_grad():
                    results = self.model(frame, device=self.device, verbose=False)
        except TypeError:
            with self.model_lock:
                with torch.no_grad():
                    results = self.model(frame, device=self.device)
        except Exception as e:
            rospy.logerr('YOLO推理失败(%s): %s', camera_role, e)
            return

        img_height, img_width = frame.shape[:2]
        img_center = np.array([img_width / 2, img_height / 2])
        detected_type = None
        valid_boxes = []

        # 下面保留原代码的类别过滤、增强过滤、中心最近目标、稳定计数逻辑。
        if len(results[0].boxes) > 0:
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]

                if camera_role == 'forward':
                    if class_name not in ['A', 'B']:
                        continue
                else:
                    if class_name not in ['A_down', 'B_down']:
                        continue

                confidence = result.conf.item()
                if self.enhanced:
                    if confidence < self.model.conf:
                        continue
                    length = abs(np.int64(result.xyxy[0][2].item() - result.xyxy[0][0].item()))
                    width = abs(np.int64(result.xyxy[0][3].item() - result.xyxy[0][1].item()))
                    if length >= filter * width or width >= filter * length:
                        continue

                x_center = (result.xyxy[0][0].item() + result.xyxy[0][2].item()) / 2
                y_center = (result.xyxy[0][1].item() + result.xyxy[0][3].item()) / 2
                box_center = np.array([x_center, y_center])
                distance = np.linalg.norm(box_center - img_center)
                valid_boxes.append((result, distance, class_name, confidence))

            if valid_boxes:
                closest = min(valid_boxes, key=lambda x: x[1])
                result, _, class_name, confidence = closest
                detected_type = class_name

                if state.last_type is None:
                    state.last_type = detected_type
                    state.stable_count = 1
                    state.switch_pending = None
                    state.no_type_count = 0
                elif detected_type == state.last_type:
                    state.stable_count += 1
                    state.switch_pending = None
                    state.no_type_count = 0
                else:
                    if state.switch_pending == detected_type:
                        state.stable_count += 1
                    else:
                        state.switch_pending = detected_type
                        state.stable_count = 1
                    state.no_type_count = 0
                    if state.stable_count >= 10:
                        state.last_type = state.switch_pending
                        state.stable_count = 1
                        state.switch_pending = None

                if detected_type == state.last_type:
                    rospy.loginfo_throttle(1.0, '[%s] 检测到的类别: %s(%.2f)', camera_role, class_name, confidence)

                    x1, y1, x2, y2 = result.xyxy[0].cpu().numpy()

                    det_msg = YoloDetection()
                    det_msg.class_name = class_name
                    det_msg.confidence = float(confidence)
                    det_msg.x = float(x1)
                    det_msg.y = float(y1)
                    det_msg.width = float(x2 - x1)
                    det_msg.height = float(y2 - y1)

                    self.publish_detection(det_msg, camera_role)
                    valid_boxes = [result]
                else:
                    valid_boxes = []
            else:
                state.no_type_count += 1
                if state.no_type_count >= 10 and state.switch_pending is not None:
                    state.last_type = state.switch_pending
                    state.stable_count = 1
                    state.switch_pending = None
        else:
            valid_boxes = []
            state.no_type_count += 1
            if state.no_type_count >= 10 and state.switch_pending is not None:
                state.last_type = state.switch_pending
                state.stable_count = 1
                state.switch_pending = None

        # 只发布前方摄像头带框图像；下方不发布图片，避免评委显示链路被下方画面抢占。
        if camera_role == 'forward':
            self.publish_forward_detection_image(frame, valid_boxes, results[0], msg.header, state)

        # 可选 OpenCV 本地窗口，也只显示前方，不影响 rqt。
        if camera_role == 'forward' and self.visualize:
            self.show_forward_image(frame, valid_boxes, results[0])

    def publish_detection(self, det_msg, camera_role):
        # 两路识别结果各发各的话题。
        if camera_role == 'down':
            self.position_pub_down.publish(det_msg)
        elif camera_role == 'forward':
            self.position_pub_forward.publish(det_msg)

        # 原 pub_topic 作为“主程序 active 视角结果”保留，受 /yolo/switch_camera 控制。
        # 单摄模式也会发布，保证原 camera_detect.launch / front.launch 不变。
        if self.single_camera or camera_role == self.active_camera:
            self.position_pub.publish(det_msg)

    # ================== 前方带框图片输出 ==================
    def _should_publish_image(self, state):
        if not self.publish_image:
            return False
        if self.image_pub.get_num_connections() <= 0:
            return False
        if self.image_pub_fps > 0:
            now = rospy.Time.now()
            if (now - state.last_image_pub_time).to_sec() < (1.0 / self.image_pub_fps):
                return False
            state.last_image_pub_time = now
        return True

    def publish_forward_detection_image(self, frame, valid_boxes, result0, header, state):
        if self.publish_front_image_only and not self._should_publish_image(state):
            return
        if not self.publish_front_image_only and not self._should_publish_image(state):
            return

        annotated_frame = frame.copy()
        self.draw_boxes(annotated_frame, valid_boxes, result0, 'FORWARD')
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            detection_msg.header = header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, '发布前方检测图像失败: %s', e)

    def show_forward_image(self, frame, valid_boxes, result0):
        if not os.environ.get('DISPLAY'):
            return
        annotated_frame = frame.copy()
        self.draw_boxes(annotated_frame, valid_boxes, result0, 'FORWARD')
        try:
            resized_frame = cv2.resize(annotated_frame, (640, 480))
            cv2.imshow('YOLOv8_Forward_Camera_View', resized_frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn_throttle(2.0, 'OpenCV imshow error: %s', e)

    def draw_boxes(self, annotated_frame, valid_boxes, result0, camera_label):
        if valid_boxes:
            # 与原逻辑一致：稳定后只画当前发布的目标框。
            for box in valid_boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                class_name = result0.names[box.cls.item()]
                conf = box.conf.item()
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(annotated_frame, f'{class_name} {conf:.2f}',
                            (int(x1), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(annotated_frame, f'Camera: {camera_label}', (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2, cv2.LINE_AA)


def main():
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
