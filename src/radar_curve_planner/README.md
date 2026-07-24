# radar_curve_planner — all-points strict 2D projection

This ROS1 Noetic catkin package generates and visualizes an obstacle-avoiding XY trajectory from `/cloud_registered` and `/mavros/local_position/pose`.

Safety policy in this version:

- The output trajectory is XY-only.
- Every finite point in `/cloud_registered` is projected to XY as an obstacle.
- No height slice, no z-band filtering, no near-start point filtering.
- No start/goal clearing.
- No initial path segment collision skipping.
- `min_collision_points=1`: one projected point inside the footprint invalidates the path.
- If the point cloud blocks the route, planning fails instead of writing a risky YAML.

Main launch:

```bash
roslaunch radar_curve_planner radar_curve_planner.launch use_rviz:=true
```

YAML output:

```text
/home/abot/kk_ws/src/radar_curve_planner/tmp/radar_generated_traj.yaml
```

Expected log keyword:

```text
projection=all_points_2d[-1000.00,1000.00]
near_ignore=OFF start_clear=OFF
```
