#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from time import time
import os
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):

        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = float(rospy.get_param('~conf', 0.5))  # 转为float
        self.visualize = rospy.get_param('~visualize', False)  # 默认False

        # which device will be used
        if rospy.get_param('/use_cpu', False):
            self.device = 'cpu'
        else:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        rospy.loginfo(f"Using device: {self.device}")
        
        self.bridge = CvBridge()
        self.model = YOLO(weight_path)
        self.model.to(self.device)
        self.model.conf = conf
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # output publishers
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov8/detection_image', Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus) and (not rospy.is_shutdown()):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        try:
            self.boundingBoxes = BoundingBoxes()
            self.boundingBoxes.header = image.header
            self.boundingBoxes.image_header = image.header
            self.getImageStatus = True

            # 使用cv_bridge安全转换图像
            try:
                # 转换为BGR格式
                self.color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(f"cv_bridge conversion failed: {e}")
                return

            # 使用YOLOv8进行预测 - 直接使用BGR图像
            results = self.model(self.color_image)
            
            self.dectshow(results, image.height, image.width)

        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    """
    def dectshow(self, results, height, width):
        try:
            # 在帧上绘制检测结果
            self.frame = results[0].plot()
            
            # 显示FPS
            if results[0].speed['inference'] > 0:
                fps = 1000.0 / results[0].speed['inference']
                cv2.putText(self.frame, f'FPS: {int(fps)}', (20, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            # 处理每个检测框 - 保持原有的坐标计算方式
            for result in results[0].boxes:
                boundingBox = BoundingBox()
                
                # 保持原有的坐标计算方式不变
                sum1 = np.int64(result.xyxy[0][0].item()/2) + np.int64(result.xyxy[0][2].item()/2)
                sum2 = np.int64(result.xyxy[0][1].item()/2) + np.int64(result.xyxy[0][3].item()/2) 
                boundingBox.xmin = sum1
                boundingBox.ymin = sum2
                boundingBox.xmax = 0
                boundingBox.ymax = 0
                
                # 获取类别名称和置信度
                boundingBox.Class = results[0].names[int(result.cls.item())]
                boundingBox.probability = float(result.conf.item())
                
                self.boundingBoxes.bounding_boxes.append(boundingBox)
            
            # 发布检测结果
            self.position_pub.publish(self.boundingBoxes)
            self.publish_image(self.frame, height, width)

            # 可视化（只在有显示环境时）
            if self.visualize and 'DISPLAY' in os.environ:
                try:
                    cv2.imshow('YOLOv8', self.frame)
                    cv2.waitKey(1)
                except Exception as e:
                    rospy.logwarn(f"Visualization failed: {e}")
                    self.visualize = False

        except Exception as e:
            rospy.logerr(f"Detection show error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    """

    def dectshow(self, results, height, width):
        try:
            # 在帧上绘制检测结果
            self.frame = results[0].plot()
            
            # 显示FPS
            if results[0].speed['inference'] > 0:
                fps = 1000.0 / results[0].speed['inference']
                cv2.putText(self.frame, f'FPS: {int(fps)}', (20, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            # ---------------------- 修改开始 ----------------------
            # 找出面积（或宽/高）最大的目标，只发布该目标
            max_size = 0
            max_box = None
            for result in results[0].boxes:
                x1, y1, x2, y2 = result.xyxy[0]
                w = (x2 - x1).item()
                h = (y2 - y1).item()
                size = max(w, h)   # 可改为 w*h 若想按面积最大
                if size > max_size:
                    max_size = size
                    max_box = result

            # 若检测到目标，则仅发布最大目标
            if max_box is not None:
                boundingBox = BoundingBox()
                sum1 = np.int64(max_box.xyxy[0][0].item()/2) + np.int64(max_box.xyxy[0][2].item()/2)
                sum2 = np.int64(max_box.xyxy[0][1].item()/2) + np.int64(max_box.xyxy[0][3].item()/2) 
                boundingBox.xmin = sum1
                boundingBox.ymin = sum2
                boundingBox.xmax = 0
                boundingBox.ymax = 0
                boundingBox.Class = results[0].names[int(max_box.cls.item())]
                boundingBox.probability = float(max_box.conf.item())
                self.boundingBoxes.bounding_boxes.append(boundingBox)
            # ---------------------- 修改结束 ----------------------

            # 发布检测结果
            self.position_pub.publish(self.boundingBoxes)
            self.publish_image(self.frame, height, width)

            # 可视化
            if self.visualize and 'DISPLAY' in os.environ:
                try:
                    cv2.imshow('YOLOv8', self.frame)
                    cv2.waitKey(1)
                except Exception as e:
                    rospy.logwarn(f"Visualization failed: {e}")
                    self.visualize = False

        except Exception as e:
            rospy.logerr(f"Detection show error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def publish_image(self, imgdata, height, width):
        try:
            image_temp = Image()
            header = Header(stamp=rospy.Time.now())
            header.frame_id = self.camera_frame
            image_temp.height = height
            image_temp.width = width
            image_temp.encoding = 'bgr8'
            image_temp.data = np.array(imgdata).tobytes()
            image_temp.header = header
            image_temp.step = width * 3
            self.image_pub.publish(image_temp)
        except Exception as e:
            rospy.logerr(f"Image publish error: {e}")


def main():
    import pkg_resources
    import ultralytics
    rospy.init_node('yolov8_ros', anonymous=True)
    
    # 输出ultralytics及其依赖包版本
    rospy.loginfo("ultralytics version: {}".format(ultralytics.__version__))
    rospy.loginfo("ultralytics dependencies:")
    
    try:
        dist = pkg_resources.get_distribution("ultralytics")
        requires = dist.requires()
        for req in requires:
            try:
                dep_dist = pkg_resources.get_distribution(req.project_name)
                rospy.loginfo("  {}: {}".format(req.project_name, dep_dist.version))
            except Exception:
                rospy.loginfo("  {}: not installed".format(req.project_name))
    except Exception as e:
        rospy.loginfo("Could not get ultralytics dependencies: {}".format(e))
    
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()


