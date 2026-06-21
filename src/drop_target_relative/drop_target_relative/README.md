# drop_target_relative

ROS1/catkin package for computing the horizontal relative offset from the current dropper outlet to a YOLO-detected drop target.

## Input

### `/yolo/detections`

Type: `drop_target_relative/DetectionArray`

Each detection is:

```text
string class_name
float32 confidence
float32 x       # bbox left-top x, pixels
float32 y       # bbox left-top y, pixels
float32 width   # bbox width, pixels
float32 height  # bbox height, pixels
```

### `/mavros/local_position/odom`

Type: `nav_msgs/Odometry`

Used for current height and yaw.

## Output

### `/drop_target_relative`

Type: `geometry_msgs/Vector3Stamped`

- `vector.x`: target x offset relative to the dropper outlet, meters
- `vector.y`: target y offset relative to the dropper outlet, meters
- `vector.z`: always 0

The output frame is controlled by `output/frame` in `config/drop_target_relative.yaml`:

- `body`: UAV body frame
- `odom`: odom/local-position frame

## Build

Copy this package into your catkin workspace:

```bash
cd ~/kk_ws/src
cp -r /path/to/drop_target_relative .
cd ~/kk_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch drop_target_relative drop_target_relative.launch
```

Check output:

```bash
rostopic echo /drop_target_relative
```

## Important parameters

Edit `config/drop_target_relative.yaml`:

```yaml
target:
  class_name: "A"

camera:
  fx: 644.0923
  fy: 649.26
  cx: 314.5237
  cy: 277.9386

dropper:
  offset_x_body: 0.0
  offset_y_body: 0.0
```

`dropper/offset_x_body` and `dropper/offset_y_body` are the dropper outlet position relative to the downward camera optical center in UAV body frame, in meters.
