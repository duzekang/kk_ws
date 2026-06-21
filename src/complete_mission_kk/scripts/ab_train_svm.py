#!/usr/bin/env python3
import os
import cv2
import numpy as np


PKG_DIR = os.path.expanduser("~/abot_ws/src/complete_mission_kk")

DATASET_DIR = os.path.join(PKG_DIR, "dataset")
MODEL_DIR = os.path.join(PKG_DIR, "model")
MODEL_PATH = os.path.join(MODEL_DIR, "ab_svm.xml")

IMG_SIZE = (64, 64)

LABEL_MAP = {
    "A": 0,
    "B": 1,
    "negative": 2
}


def preprocess(img):
    if img is None:
        raise ValueError("empty image")

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


def create_hog():
    return cv2.HOGDescriptor(
        _winSize=(64, 64),
        _blockSize=(16, 16),
        _blockStride=(8, 8),
        _cellSize=(8, 8),
        _nbins=9
    )


def extract_hog(hog, img):
    feat = hog.compute(img)
    return feat.reshape(-1)


def load_dataset():
    hog = create_hog()

    features = []
    labels = []

    for cls_name, label in LABEL_MAP.items():
        cls_dir = os.path.join(DATASET_DIR, cls_name)

        if not os.path.exists(cls_dir):
            print("[WARN] directory not found:", cls_dir)
            continue

        count = 0

        for filename in os.listdir(cls_dir):
            if not filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                continue

            path = os.path.join(cls_dir, filename)
            img = cv2.imread(path)

            if img is None:
                print("[WARN] failed to read:", path)
                continue

            try:
                binary = preprocess(img)
                feat = extract_hog(hog, binary)
            except Exception as e:
                print("[WARN] skip image:", path, e)
                continue

            features.append(feat)
            labels.append(label)
            count += 1

        print(f"[INFO] class {cls_name}: {count} images")

    if len(features) == 0:
        raise RuntimeError("No training images found. Please put images into dataset/A, dataset/B, dataset/negative.")

    features = np.array(features, dtype=np.float32)
    labels = np.array(labels, dtype=np.int32)

    return features, labels


def train():
    os.makedirs(MODEL_DIR, exist_ok=True)

    features, labels = load_dataset()

    print("[INFO] total samples:", len(labels))
    print("[INFO] feature shape:", features.shape)

    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setC(1.0)
    svm.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-6))

    svm.train(features, cv2.ml.ROW_SAMPLE, labels)
    svm.save(MODEL_PATH)

    print("[INFO] SVM model saved to:", MODEL_PATH)


if __name__ == "__main__":
    train()