#ifndef DZK_TRAJ_CONTROL_H_
#define DZK_TRAJ_CONTROL_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <XmlRpcValue.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace dzk_traj {

// MAVROS PositionTarget type_mask bits. A bit set to 1 means that field is ignored.
static constexpr uint16_t IGNORE_PX       = 1u;
static constexpr uint16_t IGNORE_PY       = 2u;
static constexpr uint16_t IGNORE_PZ       = 4u;
static constexpr uint16_t IGNORE_VX       = 8u;
static constexpr uint16_t IGNORE_VY       = 16u;
static constexpr uint16_t IGNORE_VZ       = 32u;
static constexpr uint16_t IGNORE_AFX      = 64u;
static constexpr uint16_t IGNORE_AFY      = 128u;
static constexpr uint16_t IGNORE_AFZ      = 256u;
static constexpr uint16_t FORCE_SET       = 512u;
static constexpr uint16_t IGNORE_YAW      = 1024u;
static constexpr uint16_t IGNORE_YAW_RATE = 2048u;

struct FollowerOptions {
  // 发布频率。PX4 Offboard 一般要求 setpoint 持续高频发布，建议 >= 20Hz。
  double rate_hz = 40.0;

  // 认为到达当前轨迹点的阈值。
  float arrival_xy = 0.25f;
  float arrival_z  = 0.25f;

  // 速度前馈。不是底层电机速度，而是 PositionTarget 里的 velocity feed-forward。
  float target_vel = 0.8f;
  float min_vel = 0.10f;
  float slow_down_radius = 0.8f;
  bool use_velocity_feedforward = true;

  // 偏航控制。face_to_path=true 时，机头朝向当前目标点；否则使用 fixed_yaw。
  float fixed_yaw = 0.0f;
  bool face_to_path = false;

  // 定位超时保护。odom 时间戳超时后，发布当前位置悬停点，不继续切轨迹点。
  double odom_timeout = 0.5;

  // 总超时保护；<=0 表示不启用。
  double max_run_time = 0.0;

  // 到达最后一个点后，额外悬停几帧，让飞控稳定收到终点 setpoint。
  int final_hold_ticks = 10;
};

inline float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline float norm2D(float x, float y) {
  return std::sqrt(x * x + y * y);
}

inline bool validXYPoint(const std::vector<float>& p) {
  return p.size() >= 2 && std::isfinite(p[0]) && std::isfinite(p[1]);
}

inline bool odomFresh(const nav_msgs::Odometry& odom, double timeout_sec) {
  if (timeout_sec <= 0.0) return true;
  if (odom.header.stamp.isZero()) return true;  // 有些仿真/测试数据不填 stamp，这里不强行判超时。
  return (ros::Time::now() - odom.header.stamp).toSec() <= timeout_sec;
}

inline mavros_msgs::PositionTarget makePositionTarget(
    float x, float y, float z,
    float vx, float vy, float vz,
    float yaw,
    bool use_velocity_feedforward) {
  mavros_msgs::PositionTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "map";
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  // 默认启用 position + yaw，忽略加速度和 yaw_rate。
  cmd.type_mask = IGNORE_AFX + IGNORE_AFY + IGNORE_AFZ + FORCE_SET + IGNORE_YAW_RATE;

  // 需要纯位置控制时，忽略 velocity；需要速度前馈时，不忽略 velocity。
  if (!use_velocity_feedforward) {
    cmd.type_mask += IGNORE_VX + IGNORE_VY + IGNORE_VZ;
    vx = vy = vz = 0.0f;
  }

  cmd.position.x = x;
  cmd.position.y = y;
  cmd.position.z = z;
  cmd.velocity.x = vx;
  cmd.velocity.y = vy;
  cmd.velocity.z = vz;
  cmd.yaw = yaw;
  return cmd;
}

inline void publishHoldAtCurrentPosition(
    ros::Publisher& setpoint_pub,
    const nav_msgs::Odometry& current_odom,
    float fallback_z,
    float yaw = 0.0f) {
  const float x = current_odom.pose.pose.position.x;
  const float y = current_odom.pose.pose.position.y;
  const float z = std::isfinite(static_cast<float>(current_odom.pose.pose.position.z))
                    ? static_cast<float>(current_odom.pose.pose.position.z)
                    : fallback_z;
  mavros_msgs::PositionTarget cmd = makePositionTarget(
      x, y, z, 0.0f, 0.0f, 0.0f, yaw, false);
  setpoint_pub.publish(cmd);
}

/**
 * @brief 阻塞式轨迹跟踪：逐点发布 PositionTarget，直到整条路径执行完。
 *
 * trajectory 的每个点格式为 {x, y}，坐标通常是“相对起飞点”的坐标。
 * 实际发布坐标为：
 *   world_x = trajectory[i][0] + offset_x
 *   world_y = trajectory[i][1] + offset_y
 *   world_z = height + offset_z
 *
 * 这个函数适合你现在的 mission_num 写法：进入 case 31 后调用一次，飞完再返回。
 */
inline bool followTrajectoryBlocking(
    const std::vector<std::vector<float>>& trajectory,
    float height,
    ros::Publisher& setpoint_pub,
    const nav_msgs::Odometry& current_odom,
    float offset_x = 0.0f,
    float offset_y = 0.0f,
    float offset_z = 0.0f,
    const FollowerOptions& options = FollowerOptions()) {

  if (trajectory.empty()) {
    ROS_ERROR("[traj_control] Trajectory is empty.");
    return false;
  }

  ros::Rate rate(options.rate_hz);
  const ros::Time start_time = ros::Time::now();
  std::size_t target_idx = 0;
  bool current_point_sent = false;

  ROS_INFO("[traj_control] Start trajectory follower, points=%zu, height=%.2f, offset=(%.2f, %.2f, %.2f)",
           trajectory.size(), height, offset_x, offset_y, offset_z);

  while (ros::ok() && target_idx < trajectory.size()) {
    ros::spinOnce();

    if (options.max_run_time > 0.0 &&
        (ros::Time::now() - start_time).toSec() > options.max_run_time) {
      ROS_ERROR("[traj_control] Trajectory follower timeout, target_idx=%zu/%zu.",
                target_idx, trajectory.size());
      return false;
    }

    if (!odomFresh(current_odom, options.odom_timeout)) {
      ROS_WARN_THROTTLE(1.0, "[traj_control] Odometry timeout. Hold current position.");
      publishHoldAtCurrentPosition(setpoint_pub, current_odom, height + offset_z, options.fixed_yaw);
      rate.sleep();
      continue;
    }

    if (!validXYPoint(trajectory[target_idx])) {
      ROS_WARN("[traj_control] Invalid trajectory point at index %zu, skipped.", target_idx);
      ++target_idx;
      current_point_sent = false;
      continue;
    }

    float target_x = trajectory[target_idx][0] + offset_x;
    float target_y = trajectory[target_idx][1] + offset_y;
    float target_z = height + offset_z;

    const float cx = current_odom.pose.pose.position.x;
    const float cy = current_odom.pose.pose.position.y;
    const float cz = current_odom.pose.pose.position.z;

    float dx = target_x - cx;
    float dy = target_y - cy;
    float dz = target_z - cz;
    float dist_xy = norm2D(dx, dy);

    // 只有当前点已经至少发布过一次，才允许切换到下一个点。
    if (current_point_sent && dist_xy < options.arrival_xy && std::fabs(dz) < options.arrival_z) {
      ROS_INFO_THROTTLE(0.5, "[traj_control] Arrived point %zu/%zu.",
                        target_idx + 1, trajectory.size());
      ++target_idx;
      current_point_sent = false;
      continue;
    }

    // 速度前馈：方向指向当前目标点，速度随距离减速，避免接近目标点时过冲。
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    if (options.use_velocity_feedforward && dist_xy > 1e-3f) {
      float speed = options.target_vel;
      if (options.slow_down_radius > 1e-3f && dist_xy < options.slow_down_radius) {
        speed = options.target_vel * dist_xy / options.slow_down_radius;
      }
      speed = clampFloat(speed, options.min_vel, options.target_vel);
      vx = speed * dx / dist_xy;
      vy = speed * dy / dist_xy;
      vz = clampFloat(1.0f * dz, -options.target_vel, options.target_vel);
    }

    float target_yaw = options.fixed_yaw;
    if (options.face_to_path && dist_xy > 0.05f) {
      target_yaw = std::atan2(dy, dx);
    }

    mavros_msgs::PositionTarget cmd = makePositionTarget(
        target_x, target_y, target_z,
        vx, vy, vz,
        target_yaw,
        options.use_velocity_feedforward);
    setpoint_pub.publish(cmd);
    current_point_sent = true;

    ROS_INFO_THROTTLE(0.5,
        "[traj_control] idx=%zu/%zu now=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) dist_xy=%.2f v=(%.2f, %.2f, %.2f)",
        target_idx + 1, trajectory.size(), cx, cy, cz, target_x, target_y, target_z,
        dist_xy, vx, vy, vz);

    rate.sleep();
  }

  // 终点悬停若干帧，避免函数刚退出时飞控突然丢 setpoint。
  if (!trajectory.empty()) {
    std::size_t last = trajectory.size() - 1;
    while (last > 0 && !validXYPoint(trajectory[last])) --last;
    if (validXYPoint(trajectory[last])) {
      const float final_x = trajectory[last][0] + offset_x;
      const float final_y = trajectory[last][1] + offset_y;
      const float final_z = height + offset_z;
      for (int i = 0; ros::ok() && i < options.final_hold_ticks; ++i) {
        ros::spinOnce();
        mavros_msgs::PositionTarget cmd = makePositionTarget(
            final_x, final_y, final_z,
            0.0f, 0.0f, 0.0f,
            options.fixed_yaw,
            false);
        setpoint_pub.publish(cmd);
        rate.sleep();
      }
    }
  }

  ROS_INFO("[traj_control] Trajectory completed.");
  return true;
}

/**
 * @brief 兼容你原工程调用方式的包装函数。
 *
 * 原来这样调用仍然能用：
 * traj_follower(track_1_4, height, mavros_setpoint_pos_pub, local_pos,
 *               init_position_x_take_off, init_position_y_take_off);
 *
 * 更推荐把 offset_z 也传进来：
 * traj_follower(track_1_4, height, pub, local_pos, init_x, init_y, init_z, 0.8f);
 */
inline void traj_follower(
    const std::vector<std::vector<float>>& trajectory,
    float height,
    ros::Publisher& setpoint_pub,
    const nav_msgs::Odometry& current_odom,
    float offset_x = 0.0f,
    float offset_y = 0.0f,
    float offset_z = 0.0f,
    float target_vel = 0.8f) {
  FollowerOptions options;
  options.target_vel = target_vel;
  options.fixed_yaw = 0.0f;
  options.face_to_path = false;
  followTrajectoryBlocking(trajectory, height, setpoint_pub, current_odom,
                           offset_x, offset_y, offset_z, options);
}

inline bool xmlRpcNumberToDouble(XmlRpc::XmlRpcValue& value, double& out) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    out = static_cast<double>(value);
    return true;
  }
  return false;
}

/**
 * @brief 从 rosparam 读取轨迹。兼容两种 YAML：
 *
 * 写法 1：
 * trajectory:
 *   - {x: 0.0, y: 0.0}
 *   - {x: 0.5, y: 0.2}
 *
 * 写法 2：
 * trajectory:
 *   - [0.0, 0.0]
 *   - [0.5, 0.2]
 */
inline bool getTrajectoryFromParam(
    ros::NodeHandle& nh,
    const std::string& param_name,
    std::vector<std::vector<float>>& out_traj) {
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam(param_name, list)) {
    ROS_ERROR("[traj_control] Failed to get parameter: %s", param_name.c_str());
    return false;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("[traj_control] Parameter %s is not an array.", param_name.c_str());
    return false;
  }

  out_traj.clear();
  for (int i = 0; i < list.size(); ++i) {
    XmlRpc::XmlRpcValue& item = list[i];
    double x = 0.0;
    double y = 0.0;
    bool ok = false;

    if (item.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      if (item.hasMember("x") && item.hasMember("y")) {
        ok = xmlRpcNumberToDouble(item["x"], x) && xmlRpcNumberToDouble(item["y"], y);
      }
    } else if (item.getType() == XmlRpc::XmlRpcValue::TypeArray && item.size() >= 2) {
      ok = xmlRpcNumberToDouble(item[0], x) && xmlRpcNumberToDouble(item[1], y);
    }

    if (!ok) {
      ROS_ERROR("[traj_control] Invalid trajectory point at %s[%d].", param_name.c_str(), i);
      out_traj.clear();
      return false;
    }
    out_traj.push_back({static_cast<float>(x), static_cast<float>(y)});
  }

  ROS_INFO("[traj_control] Loaded %zu points from param: %s", out_traj.size(), param_name.c_str());
  return true;
}

}  // namespace dzk_traj

// 为了兼容原代码中不带命名空间的调用，提供两个全局包装。
inline void traj_follower(
    const std::vector<std::vector<float>>& trajectory,
    float height,
    ros::Publisher& setpoint_pub,
    const nav_msgs::Odometry& current_odom,
    float offset_x = 0.0f,
    float offset_y = 0.0f,
    float offset_z = 0.0f,
    float target_vel = 0.8f) {
  dzk_traj::traj_follower(trajectory, height, setpoint_pub, current_odom,
                          offset_x, offset_y, offset_z, target_vel);
}

inline bool getTrajectoryFromParam(
    ros::NodeHandle& nh,
    const std::string& param_name,
    std::vector<std::vector<float>>& out_traj) {
  return dzk_traj::getTrajectoryFromParam(nh, param_name, out_traj);
}

#endif  // DZK_TRAJ_CONTROL_H_
