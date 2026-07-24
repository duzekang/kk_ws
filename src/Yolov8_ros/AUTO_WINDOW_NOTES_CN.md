# 前方带框图自动弹窗说明

本版本在完整双摄 `camera_detect_dual.launch` 中默认 `visualize:=true`，因此启动完整功能后会自动打开 OpenCV 窗口显示前方摄像头带框画面。

## 完整功能启动

```bash
roslaunch yolov8_ros camera_detect_dual.launch image_pub_fps:=10
```

默认逻辑：

- 订阅下方摄像头：`/down/usb_cam/image_raw`
- 订阅前方摄像头：`/forward/usb_cam_front/image_raw`
- 订阅主程序切换：`/yolo/switch_camera`
- 发布 active 识别：`/yolo/detections`
- 发布下方识别：`/yolo/detections_down`
- 发布前方识别：`/yolo/detections_forward`
- 发布前方带框图：`/yolov8_detection/image_result`
- 自动弹窗显示前方带框图：`YOLOv8_Forward_Camera_View`

下方摄像头仍然识别，但不会发布下方带框图片，也不会弹出下方窗口。

## 关闭自动窗口

```bash
roslaunch yolov8_ros camera_detect_dual.launch visualize:=false
```

## 修改窗口名

```bash
roslaunch yolov8_ros camera_detect_dual.launch window_name:=front_view
```
