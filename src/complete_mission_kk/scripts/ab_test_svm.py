#!/usr/bin/env python3
import os
import cv2
import numpy as np

PKG_DIR = os.path.expanduser("~/abot_ws/src/complete_mission_kk")

TEST_DIR = os.path.join(PKG_DIR, "test")
RESULT_DIR = os.path.join(PKG_DIR, "result")
MODEL_PATH = os.path.join(PKG_DIR, "model", "ab_svm.xml")

IMG_SIZE = (64, 64)

LABEL_NAME = {
    0: "A",
    1: "B",
    2: "N"
}


def create_hog():
    return cv2.HOGDescriptor(
        _winSize=(64, 64),
        _blockSize=(16, 16),
        _blockStride=(8, 8),
        _cellSize=(8, 8),
        _nbins=9
    )


def preprocess_roi(img):
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


def extract_hog(hog, img):
    feat = hog.compute(img)
    return feat.reshape(1, -1).astype(np.float32)


def predict_roi(svm, hog, roi):
    binary = preprocess_roi(roi)
    feat = extract_hog(hog, binary)

    _, pred = svm.predict(feat)
    label = int(pred[0][0])

    try:
        _, raw = svm.predict(feat, flags=cv2.ml.StatModel_RAW_OUTPUT)
        raw_score = abs(float(raw[0][0]))
        confidence = min(0.99, raw_score / 3.0)
    except Exception:
        confidence = 0.90

    return LABEL_NAME.get(label, "N"), confidence


def find_candidates(frame, min_area=300, max_area=200000):
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

        if area < min_area:
            continue
        if area > max_area:
            continue
        if w < 10 or h < 10:
            continue

        ratio = float(w) / float(h)
        if ratio < 0.2 or ratio > 2.0:
            continue

        candidates.append((x, y, w, h))

    return candidates


def detect_image(frame, svm, hog, conf_threshold=0.80):
    frame_h, frame_w = frame.shape[:2]
    image_cx = frame_w // 2
    image_cy = frame_h // 2

    candidates = find_candidates(frame)

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

        letter, conf = predict_roi(svm, hog, roi)

        if letter in ["A", "B"] and conf > best_conf:
            best_letter = letter
            best_conf = conf
            best_rect = (x1, y1, x2 - x1, y2 - y1)

    if best_rect is None or best_conf < conf_threshold:
        return "N", 0.0, None, 0, 0

    x, y, w, h = best_rect
    center_x = x + w // 2
    center_y = y + h // 2

    dx = center_x - image_cx
    dy = center_y - image_cy

    return best_letter, best_conf, best_rect, dx, dy


def draw_result(frame, letter, conf, rect, dx, dy):
    out = frame.copy()

    h, w = out.shape[:2]
    image_cx = w // 2
    image_cy = h // 2

    cv2.circle(out, (image_cx, image_cy), 5, (255, 0, 0), -1)

    if rect is not None:
        x, y, bw, bh = rect
        center_x = x + bw // 2
        center_y = y + bh // 2

        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(out, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.line(out, (image_cx, image_cy), (center_x, center_y), (255, 0, 0), 2)

        cv2.putText(
            out,
            f"{letter} conf={conf:.2f}",
            (x, max(25, y - 45)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            out,
            f"dx={dx} dy={dy}",
            (x, max(25, y - 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )
    else:
        cv2.putText(
            out,
            "N conf=0.00",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2
        )

    return out


def main():
    os.makedirs(RESULT_DIR, exist_ok=True)

    if not os.path.exists(MODEL_PATH):
        raise RuntimeError(f"model not found: {MODEL_PATH}")

    if not os.path.exists(TEST_DIR):
        raise RuntimeError(f"test dir not found: {TEST_DIR}")

    svm = cv2.ml.SVM_load(MODEL_PATH)
    hog = create_hog()

    image_files = [
        f for f in os.listdir(TEST_DIR)
        if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
    ]

    print("[INFO] test images:", len(image_files))

    summary_path = os.path.join(RESULT_DIR, "summary.txt")

    count_a = 0
    count_b = 0
    count_n = 0

    with open(summary_path, "w") as fw:
        fw.write("filename,letter,confidence,dx,dy\n")

        for filename in image_files:
            path = os.path.join(TEST_DIR, filename)
            frame = cv2.imread(path)

            if frame is None:
                print("[WARN] failed to read:", path)
                continue

            letter, conf, rect, dx, dy = detect_image(frame, svm, hog)

            if letter == "A":
                count_a += 1
            elif letter == "B":
                count_b += 1
            else:
                count_n += 1

            out = draw_result(frame, letter, conf, rect, dx, dy)

            save_path = os.path.join(RESULT_DIR, filename)
            cv2.imwrite(save_path, out)

            fw.write(f"{filename},{letter},{conf:.2f},{dx},{dy}\n")

            print(f"[RESULT] {filename}: {letter}, conf={conf:.2f}, dx={dx}, dy={dy}")

        fw.write("\n")
        fw.write(f"A_count,{count_a}\n")
        fw.write(f"B_count,{count_b}\n")
        fw.write(f"N_count,{count_n}\n")
        fw.write(f"total,{count_a + count_b + count_n}\n")

    print("[INFO] result images saved to:", RESULT_DIR)
    print("[INFO] summary saved to:", summary_path)


if __name__ == "__main__":
    main()