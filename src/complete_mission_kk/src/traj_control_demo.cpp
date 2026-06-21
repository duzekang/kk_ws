#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include "traj_control.h"

nav_msgs::Odometry g_odom;
bool g_has_odom = false;

void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  g_odom = *msg;
  g_has_odom = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_control_demo");
  ros::NodeHandle nh;

  ros::Publisher setpoint_pub =
      nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 20);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 20, odomCb);

  ros::Rate wait_rate(20.0);
  while (ros::ok() && !g_has_odom) {
    ROS_INFO_THROTTLE(1.0, "Waiting for odometry...");
    ros::spinOnce();
    wait_rate.sleep();
  }

  const float init_x = g_odom.pose.pose.position.x;
  const float init_y = g_odom.pose.pose.position.y;
  const float init_z = g_odom.pose.pose.position.z;

  // 示例轨迹：相对起飞点的 XY 坐标。实际发布时会加 init_x / init_y / init_z。
  std::vector<std::vector<float>> path = {
      {0.0f, 0.0f},
      {0.5f, 0.0f},
      {1.0f, 0.25f},
      {1.5f, 0.25f},
      {2.0f, 0.0f}
  };

  // 也可以从 rosparam 读取；读取失败时使用上面的内置 path。
  std::vector<std::vector<float>> loaded_path;
  if (getTrajectoryFromParam(nh, "demo_avoid_path/trajectory", loaded_path)) {
    path = loaded_path;
  }

  dzk_traj::FollowerOptions opt;
  opt.target_vel = 0.6f;
  opt.arrival_xy = 0.25f;
  opt.arrival_z = 0.25f;
  opt.face_to_path = false;     // true 表示机头朝轨迹方向
  opt.fixed_yaw = 0.0f;

  const float height = 0.9f;    // 相对起飞高度

  // 推荐写法：把 init_z 也传进去，使 z = height + init_z。
  dzk_traj::followTrajectoryBlocking(path, height, setpoint_pub, g_odom,
                                     init_x, init_y, init_z, opt);

  ROS_INFO("Demo trajectory finished.");
  return 0;
}
