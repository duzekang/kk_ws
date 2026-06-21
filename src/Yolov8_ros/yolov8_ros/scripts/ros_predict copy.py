#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# 引入新定义的自定义消息
from yolov8_ros_msgs.msg import YoloDetection 
from std_msgs.msg import Header
from std_msgs.msg import String
import numpy as np

filter = 2

class YOLODetector:
    def __init__(self):
        self.last_type = None
        self.stable_count = 0
        self.switch_pending = None
        self.no_type_count = 0
        
        
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 默认权重路径改为 abdown.pt
        weight_path = rospy.get_param('~weight_path', '/home/abot/kk_ws/src/Yolov8_ros/yolov8_ros/weights/abdown.pt')
        conf = rospy.get_param('~conf', 0.5)
        use_cpu = rospy.get_param('~use_cpu', False)
        
        self.device = 'cpu' if use_cpu else ('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.model = YOLO(weight_path)
        if torch.cuda.is_available() and not use_cpu:
            self.model.to(self.device)
        self.model.conf = conf
        
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/down/usb_cam/image_raw')
        
        # 话题改为要求发布的 /yolo/detections，消息类型改为 YoloDetection
        pub_topic = rospy.get_param('~pub_topic', '/yolo/detections')
        self.position_pub = rospy.Publisher(pub_topic, YoloDetection, queue_size=1)
        
        self.visualize = rospy.get_param('~visualize', False)
        self.enhanced = rospy.get_param('~enhanced', True)
        image_pub_topic = rospy.get_param('~image_pub_topic', '/yolov8_down/detection_image')
        
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=1)
        
        print(f"加载模型: {weight_path}")
        print(f"使用设备: {self.device}")
        print(f"发布边界框话题: {pub_topic}")
        print("开始实时检测，按Ctrl+C退出...")
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")
            return
        
        results = self.model(frame, device=self.device)
        
        img_height, img_width = frame.shape[:2]
        img_center = np.array([img_width / 2, img_height / 2])
        detected_type = None
        valid_boxes = []
        
        if len(results[0].boxes) > 0:
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]
                # 仅筛选 A 和 B 标签
                if class_name not in ["A", "B"]:
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
                
                if self.last_type is None:
                    self.last_type = detected_type
                    self.stable_count = 1
                    self.switch_pending = None
                    self.no_type_count = 0
                elif detected_type == self.last_type:
                    self.stable_count += 1
                    self.switch_pending = None
                    self.no_type_count = 0
                else:
                    if self.switch_pending == detected_type:
                        self.stable_count += 1
                    else:
                        self.switch_pending = detected_type
                        self.stable_count = 1
                    self.no_type_count = 0
                    if self.stable_count >= 10:
                        self.last_type = self.switch_pending
                        self.stable_count = 1
                        self.switch_pending = None
                        
                if detected_type == self.last_type:
                    print(f"检测到的类别: {class_name}({confidence:.2f})")
                    
                    # 提取左上角坐标和宽高
                    x1, y1, x2, y2 = result.xyxy[0].cpu().numpy()
                    
                    # 组装自定义消息 YoloDetection
                    det_msg = YoloDetection()
                    det_msg.class_name = class_name
                    det_msg.confidence = float(confidence)
                    det_msg.x = float(x1)
                    det_msg.y = float(y1)
                    det_msg.width = float(x2 - x1)
                    det_msg.height = float(y2 - y1)
                    
                    self.position_pub.publish(det_msg)
                    valid_boxes = [result] 
                else:
                    valid_boxes = []
            else:
                self.no_type_count += 1
                if self.no_type_count >= 10 and self.switch_pending is not None:
                    self.last_type = self.switch_pending
                    self.stable_count = 1
                    self.switch_pending = None
        else:
            valid_boxes = []
            self.no_type_count += 1
            if self.no_type_count >= 10 and self.switch_pending is not None:
                self.last_type = self.switch_pending
                self.stable_count = 1
                self.switch_pending = None
        
        # 可视化部分保留原样...
        if valid_boxes:
            annotated_frame = frame.copy()
            box = valid_boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            class_name = results[0].names[box.cls.item()]
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        else:
            annotated_frame = results[0].plot()
        
        self.publish_detection_image(annotated_frame, msg.header)
        
        if self.visualize:
            try:
                import os
                if os.environ.get('DISPLAY'):
                    resized_frame = cv2.resize(annotated_frame, (640, 480))
                    cv2.imshow('YOLOv8_Detection', resized_frame)
                    cv2.waitKey(1)
            except Exception:
                pass

    def publish_detection_image(self, image, header):
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            detection_msg.header = header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            pass

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