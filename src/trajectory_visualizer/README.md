# trajectory_visualizer

根据外界 `std_msgs/Int32` 路线信号，从包内 `config/lll.yaml` 读取对应二维轨迹点，并发布 RViz 可视化。

## 输入信号

默认订阅：

```text
/trajectory_visualizer/route      std_msgs/Int32
```

兼容订阅：

```text
/trajectory_visualizer/select_id  std_msgs/Int32
```

路线映射：

```text
103 -> traj1_3
104 -> traj1_4
203 -> traj2_3
204 -> traj2_4
```

回归和前进路线一致，不处理 `*_back`。

主程序示例：

```cpp
int route = 0;
if(msg->obs_1 && msg->obs_3) route = 103;
if(msg->obs_1 && msg->obs_4) route = 104;
if(msg->obs_2 && msg->obs_3) route = 203;
if(msg->obs_2 && msg->obs_4) route = 204;
std_msgs::Int32 out;
out.data = route;
traj_pub.publish(out);
```

`traj_pub` 发布到 `/trajectory_visualizer/route`。

## 输出

```text
/radar_curve_planner/yaml_path       nav_msgs/Path
/radar_curve_planner/yaml_markers    visualization_msgs/MarkerArray
/competition_trajectory/path         nav_msgs/Path
/competition_trajectory/markers      visualization_msgs/MarkerArray
/trajectory_visualizer/selected_type std_msgs/String
```

## 运行

```bash
cd /home/abot/kk_ws
source devel/setup.bash
roslaunch trajectory_visualizer trajectory_visualizer.launch
```

RViz 已经打开时，只要之前加过 `/radar_curve_planner/yaml_path` 和 `/radar_curve_planner/yaml_markers`，收到路线信号后会自动刷新显示。

## 测试

```bash
rostopic pub -1 /trajectory_visualizer/route std_msgs/Int32 "data: 104"
```

或者：

```bash
roslaunch trajectory_visualizer demo_signal.launch route:=104
```
