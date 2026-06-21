#!/bin/bash

SESSION="kk_system"

# -------------------------------
# Step 1: 清理并创建 tmux 会话
# -------------------------------
echo " 正在清理旧的 tmux 会话..."
tmux kill-session -t $SESSION 2>/dev/null || true

echo "创建新的 tmux 会话: $SESSION"
tmux new-session -d -s $SESSION -n bringup

# -------------------------------
# Step 2: 加载 ROS 环境变量
# -------------------------------
source /opt/ros/noetic/setup.bash
source ~/kk_ws/devel/setup.bash

if [ $? -ne 0 ]; then
  echo " 环境变量加载失败，请检查 workspace 路径"
  exit 1
fi

# -------------------------------
# Step 3: 窗口 0: 核心节点（Core Nodes）
# -------------------------------

echo "ROS 环境加载完成"

# Pane 0.0: roscore
tmux send-keys -t $SESSION:0 'echo " 启动 roscore"; roscore' C-m

# Pane 0.1: beginning
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 'sleep 4; roslaunch abot_bringup location.launch' C-m

# Pane 0.2: obstacle_detector
tmux split-window -h -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 'sleep 4; source ~/kk_ws/devel/setup.bash; roslaunch obstacle_detector obstacle_detector.launch' C-m

# Pane 0.3: laser
tmux split-window -h -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 'echo " 启动激光笔/投货..."; sleep 3; source ~/kk_ws/devel/setup.bash; roslaunch laser_servo_control laser_servo_control.launch' C-m

# #Pane 0.4: traj_view
tmux split-window -h -t $SESSION:0.3
tmux send-keys -t $SESSION:0.4 'echo " 启动检测投货点..."; sleep 3; source ~/kk_ws/devel/setup.bash; roslaunch yolov8_ros camera_detect_dual.launch' C-m

# 设置平铺布局
tmux select-layout -t $SESSION:0 tiled


# -------------------------------
# Step 4: 窗口 1: 实时监控面板
# -------------------------------
tmux new-window -t $SESSION:1 -n monitor

tmux send-keys -t $SESSION:1 'echo "无人机位姿:"; sleep 4; rostopic echo /mavros/local_position/pose' C-m

tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 'sleep 4; source ~/kk_ws/devel/setup.bash; rostopic echo /yolo/detections ' C-m

tmux split-window -h -t $SESSION:1.1
tmux send-keys -t $SESSION:1.2 'sleep 12; source ~/kk_ws/devel/setup.bash; roslaunch trajectory_visualizer trajectory_visualizer.launch use_rviz:=true' C-m

tmux split-window -h -t $SESSION:1.2
tmux send-keys -t $SESSION:1.3 'sleep 8; source ~/kk_ws/devel/setup.bash; roslaunch complete_mission_kk lll.launch' C-m

tmux select-layout -t $SESSION:1 tiled


# -------------------------------
# Step 5: 连接到主窗口
# -------------------------------
tmux select-window -t $SESSION:1
echo " 所有节点已启动！正在连接到 tmux 会话..."
tmux attach-session -t $SESSION
