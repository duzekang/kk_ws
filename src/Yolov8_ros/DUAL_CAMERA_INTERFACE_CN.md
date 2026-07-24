# 双摄接口说明

## 启动完整功能

```bash
roslaunch yolov8_ros camera_detect_dual.launch image_pub_fps:=10
```

默认会启动：

- 下方摄像头：`/dev/video0` -> `/down/usb_cam/image_raw`
- 前方摄像头：`/dev/video2` -> `/forward/usb_cam_front/image_raw`
- YOLO 双路识别节点：同时识别前方和下方
- 自动 OpenCV 窗口：只显示前方摄像头带框画面，窗口名 `YOLOv8_Forward_Camera_View`

## 输入

| 类型 | 话题 | 消息类型 |
|---|---|---|
| 下方摄像头图像 | `/down/usb_cam/image_raw` | `sensor_msgs/Image` |
| 前方摄像头图像 | `/forward/usb_cam_front/image_raw` | `sensor_msgs/Image` |
| 主程序切换信息 | `/yolo/switch_camera` | `std_msgs/String`，内容为 `down` 或 `forward` |

## 输出

| 类型 | 话题 | 消息类型 | 说明 |
|---|---|---|---|
| 主程序 active 识别结果 | `/yolo/detections` | `yolov8_ros_msgs/YoloDetection` | 受 `/yolo/switch_camera` 控制，兼容原主程序 |
| 下方识别结果 | `/yolo/detections_down` | `yolov8_ros_msgs/YoloDetection` | 始终发布下方 `A_down/B_down` |
| 前方识别结果 | `/yolo/detections_forward` | `yolov8_ros_msgs/YoloDetection` | 始终发布前方 `A/B` |
| 前方带框图像 | `/yolov8_detection/image_result` | `sensor_msgs/Image` | 仍保留发布，方便调试订阅 |
| 前方自动窗口 | `YOLOv8_Forward_Camera_View` | OpenCV 窗口 | 默认自动打开，不需要手动订阅 |

## 订阅示例

```bash
rostopic echo /yolo/detections
rostopic echo /yolo/detections_down
rostopic echo /yolo/detections_forward
# 现在默认会自动弹窗；只有需要额外调试时才开 rqt
rqt_image_view /yolov8_detection/image_result
```

## 切换示例

```bash
rostopic pub /yolo/switch_camera std_msgs/String "data: 'down'" -1
rostopic pub /yolo/switch_camera std_msgs/String "data: 'forward'" -1
```

注意：切换只影响 `/yolo/detections` 这个给主程序用的 active 输出，不会停止另一侧识别。前方/下方各自的话题仍然都会发布。

## 卡顿时推荐参数

```bash
roslaunch yolov8_ros camera_detect_dual.launch image_pub_fps:=10 frame_skip_forward:=1 frame_skip_down:=2
```

- `image_pub_fps:=10`：只限制前方带框图像发布频率，不限制识别结果话题。
- `frame_skip_down:=2`：下方隔帧识别，降低负载。

## 自动窗口开关

默认打开前方带框窗口。关闭窗口显示：

```bash
roslaunch yolov8_ros camera_detect_dual.launch visualize:=false
```

改窗口名：

```bash
roslaunch yolov8_ros camera_detect_dual.launch window_name:=front_view
```
