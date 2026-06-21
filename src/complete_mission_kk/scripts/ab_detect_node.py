#!/usr/bin/env python3
import os
import cv2
import rospy
import numpy as np
from std_msgs.msg import String


IMG_SIZE = (64, 64)

LABEL_NAME = {
    0: "A",
    1: "B",
    2: "N"
}


class ABDetector:
    def __init__(self):
        default_pkg_dir = os.path.expanduser("~/abot_ws/src/complete_mission_kk")

        self.model_path = rospy.get_param(
            "~model_path",
            os.path.join(default_pkg_dir, "model", "ab_svm.xml")
        )

        self.camera_id = rospy.get_param("~camera_id", 0)
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.80))
        self.min_area = int(rospy.get_param("~min_area", 300))
        self.max_area = int(rospy.get_param("~max_area", 200000))
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))

        self.fx = float(rospy.get_param("~fx", 620.0))
        self.fy = float(rospy.get_param("~fy", 618.0))
        self.cx0 = float(rospy.get_param("~cx", 320.0))
        self.cy0 = float(rospy.get_param("~cy", 240.0))
        self.target_real_height = float(rospy.get_param("~target_real_height", 0.20))

        self.pub = rospy.Publisher("/ab_detect/result", String, queue_size=10)

        if not os.path.exists(self.model_path):
            raise RuntimeError("SVM model not found: " + self.model_path)

        self.svm = cv2.ml.SVM_load(self.model_path)

        self.hog = cv2.HOGDescriptor(
            _winSize=(64, 64),
            _blockSize=(16, 16),
            _blockStride=(8, 8),
            _cellSize=(8, 8),
            _nbins=9
        )

        self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera_id=" + str(self.camera_id))

        rospy.loginfo("AB detector started")
        rospy.loginfo("model_path: %s", self.model_path)
        rospy.loginfo("camera_id: %s", str(self.camera_id))
        rospy.loginfo("confidence_threshold: %.2f", self.conf_threshold)

    def preprocess_roi(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, IMG_SIZE)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        _, binary = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        return binary

    def extract_hog(self, img):
        feat = self.hog.compute(img)
        return feat.reshape(1, -1).astype(np.float32)

    def predict_roi(self, roi):
        binary = self.preprocess_roi(roi)
        feat = self.extract_hog(binary)

        _, pred = self.svm.predict(feat)
        label = int(pred[0][0])

        try:
            _, raw = self.svm.predict(feat, flags=cv2.ml.StatModel_RAW_OUTPUT)
            raw_score = abs(float(raw[0][0]))
            confidence = min(0.99, raw_score / 3.0)
        except Exception:
            confidence = 0.90

        letter = LABEL_NAME.get(label, "N")
        return letter, confidence

    def find_candidates(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        _, binary = cv2.threshold(
            blur,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = w * h

            if area < self.min_area:
                continue

            if area > self.max_area:
                continue

            if w < 10 or h < 10:
                continue

            ratio = float(w) / float(h)

            if ratio < 0.2 or ratio > 2.0:
                continue

            candidates.append((x, y, w, h))

        return candidates

    def estimate_position(self, center_x, center_y, bbox_h):
        if bbox_h <= 0:
            return 0.0, 0.0, 0.0

        z = self.target_real_height * self.fy / float(bbox_h)
        x = (float(center_x) - self.cx0) * z / self.fx
        y = (float(center_y) - self.cy0) * z / self.fy

        return x, y, z

    def publish_none(self):
        self.pub.publish("N,0.00,-1,-1,0,0,0.000,0.000,0.000")

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.logwarn("Failed to read camera frame")
                self.publish_none()
                rate.sleep()
                continue

            frame_h, frame_w = frame.shape[:2]
            image_cx = frame_w // 2
            image_cy = frame_h // 2

            candidates = self.find_candidates(frame)

            best_letter = "N"
            best_conf = 0.0
            best_rect = None

            for rect in candidates:
                x, y, w, h = rect

                pad = 5
                x1 = max(0, x - pad)
                y1 = max(0, y - pad)
                x2 = min(frame_w, x + w + pad)
                y2 = min(frame_h, y + h + pad)

                roi = frame[y1:y2, x1:x2]

                if roi.size == 0:
                    continue

                letter, conf = self.predict_roi(roi)

                if letter in ["A", "B"] and conf > best_conf:
                    best_letter = letter
                    best_conf = conf
                    best_rect = (x1, y1, x2 - x1, y2 - y1)

            if best_rect is not None and best_conf >= self.conf_threshold:
                x, y, w, h = best_rect

                center_x = x + w // 2
                center_y = y + h // 2

                dx_pixel = center_x - image_cx
                dy_pixel = center_y - image_cy

                pos_x, pos_y, pos_z = self.estimate_position(center_x, center_y, h)

                msg = (
                    f"{best_letter},{best_conf:.2f},"
                    f"{center_x},{center_y},"
                    f"{dx_pixel},{dy_pixel},"
                    f"{pos_x:.3f},{pos_y:.3f},{pos_z:.3f}"
                )

                self.pub.publish(msg)

                if self.show_window:
                    cv2.rectangle(
                        frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        frame,
                        (center_x, center_y),
                        4,
                        (0, 0, 255),
                        -1
                    )

                    cv2.circle(
                        frame,
                        (image_cx, image_cy),
                        4,
                        (255, 0, 0),
                        -1
                    )

                    cv2.line(
                        frame,
                        (image_cx, image_cy),
                        (center_x, center_y),
                        (255, 0, 0),
                        2
                    )

                    text1 = f"{best_letter} conf={best_conf:.2f}"
                    text2 = f"dx={dx_pixel} dy={dy_pixel}"
                    text3 = f"pos=({pos_x:.2f},{pos_y:.2f},{pos_z:.2f})m"

                    cv2.putText(
                        frame,
                        text1,
                        (x, max(20, y - 45)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        text2,
                        (x, max(20, y - 25)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        text3,
                        (x, max(20, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

            else:
                self.publish_none()

            if self.show_window:
                cv2.imshow("AB Detect", frame)
                cv2.waitKey(1)

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("ab_detect_node")

    try:
        node = ABDetector()
        node.run()
    except Exception as e:
        rospy.logerr("AB detector error: %s", str(e))