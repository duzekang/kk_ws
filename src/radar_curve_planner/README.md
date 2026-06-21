# radar_curve_planner

Smooth EGO-lite ROS1/Noetic trajectory YAML generator with RViz visualization.

## Important behavior

This version plans using **height-slice 2D projection** by default:

1. Take only point-cloud points near a configurable height slice.
2. Project those points to the XY plane.
3. Inflate the 2D obstacles by the drone safety radius.
4. Run A* + smoothing within the virtual wall boundary.

This is intended for an XY-only follower: the path does not try to fly over obstacles, but it also avoids letting ground/low-height noise fill the entire 2D map.

Default slice:

```yaml
project_pointcloud_to_2d: true
use_height_slice_projection: true
projection_slice_center_z: 0.80
projection_slice_half_width: 0.15
projection_min_z: 0.65
projection_max_z: 0.95
```

So the planner uses points in `[0.65, 0.95]`, equivalent to `z = 0.8 ± 0.15 m`, then compresses them into 2D obstacles.

If the obstacle point cloud is sparse at 0.8 m, widen the slice:

```yaml
projection_slice_half_width: 0.20
```

If too many points are still included, narrow it:

```yaml
projection_slice_half_width: 0.10
```

Keep the parameters under `radar_midpoint_node` and `curve_yaml_writer_node` consistent.

## Run

```bash
cd /home/abot/kk_ws
catkin_make --pkg radar_curve_planner
source devel/setup.bash
roslaunch radar_curve_planner radar_curve_planner.launch use_rviz:=true
```

## Visualize existing YAML

```bash
roslaunch radar_curve_planner view_yaml_traj.launch use_rviz:=true
```

RViz Fixed Frame should be `camera_init` unless your point cloud uses another frame.

## Output

```text
/home/abot/kk_ws/src/radar_curve_planner/tmp/radar_generated_traj.yaml
```

## Main topics

- `/cloud_registered`: input point cloud
- `/mavros/local_position/pose`: current pose
- `/radar_curve_planner/mid_points`: smoothed midpoint path
- `/radar_curve_planner/fitted_path`: final YAML writer path
- `/radar_curve_planner/curve_markers`: final trajectory markers
- `/radar_curve_planner/yaml_path`: YAML visualizer path
- `/radar_curve_planner/yaml_markers`: YAML visualizer markers
