#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import threading
import sys
import select

class AutoCaptureNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('auto_capture_node', anonymous=True)
        
        # 参数获取
        topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')  # 默认图像话题
        self.save_path = os.path.expanduser(rospy.get_param('~save_path', '~/abot_ws/src/complete_mission_kk/dataset/A'))  # 默认保存路径
        self.max_saves = rospy.get_param('~max_saves', 100)  # 最大保存数量
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        
        # 初始化工具和状态
        self.bridge = CvBridge()
        self.save_counter = 0
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # 订阅图像话题
        self.subscription = rospy.Subscriber(topic, Image, self.image_callback, queue_size=10)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        rospy.loginfo(
            f"节点已启动 | 订阅话题: {topic}\n"
            f"保存路径: {self.save_path} | 最大保存数: {self.max_saves}\n"
            f"按回车键拍摄图片，按Ctrl+C退出程序")

    def save_image(self, image):
        """保存图像到指定路径"""
        if self.save_counter >= self.max_saves:
            rospy.logwarn("达到最大保存数量限制，停止保存")
            return False
        
        # 生成带时间戳和序号的文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"A_{self.save_counter:04d}_{timestamp}.png"
        full_path = os.path.join(self.save_path, filename)
        
        # 保存图像
        cv2.imwrite(full_path, image)
        self.save_counter += 1
        rospy.loginfo(f"截图保存: {full_path} (总计: {self.save_counter}/{self.max_saves})")
        return True

    def image_callback(self, msg):
        try:
            # 转换ROS消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.latest_image = cv_image
                
        except Exception as e:
            rospy.logerr(f"图像处理错误: {str(e)}")

    def keyboard_listener(self):
        """监听键盘输入的线程"""
        while not rospy.is_shutdown():
            # 等待用户按下回车键
            try:
                # 使用select实现非阻塞输入检测
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline()
                    if line:  # 检测到回车键
                        self.capture_image()
            except Exception as e:
                rospy.logerr(f"键盘监听错误: {str(e)}")
                time.sleep(0.1)

    def capture_image(self):
        """捕获当前图像"""
        with self.image_lock:
            if self.latest_image is not None:
                self.save_image(self.latest_image)
            else:
                rospy.logwarn("尚未收到图像数据，无法拍摄")

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            rospy.loginfo(f"程序结束，总计保存截图: {self.save_counter}张")

if __name__ == '__main__':
    node = AutoCaptureNode()
    node.run()