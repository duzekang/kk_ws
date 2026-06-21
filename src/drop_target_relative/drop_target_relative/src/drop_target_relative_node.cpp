#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <drop_target_relative/DetectionArray.h>
#include <drop_target_relative/Detection.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>

class DropTargetRelativeNode {
public:
  DropTargetRelativeNode() : nh_(), pnh_("~") {
    loadParams();

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &DropTargetRelativeNode::odomCallback, this);
    det_sub_ = nh_.subscribe(detection_topic_, 10, &DropTargetRelativeNode::detectionCallback, this);
    rel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(relative_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                            &DropTargetRelativeNode::timerCallback,
                            this);

    ROS_INFO("[drop_target_relative] started. detection_topic=%s odom_topic=%s relative_topic=%s target_class=%s output_frame=%s",
             detection_topic_.c_str(), odom_topic_.c_str(), relative_topic_.c_str(),
             target_class_.c_str(), output_frame_.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber det_sub_;
  ros::Publisher rel_pub_;
  ros::Timer timer_;

  // Topics
  std::string detection_topic_;
  std::string odom_topic_;
  std::string relative_topic_;

  // Frames and output
  std::string output_frame_;     // "body" or "odom"
  std::string body_frame_id_;
  std::string odom_frame_id_;

  // Target selection
  std::string target_class_;
  double min_confidence_ = 0.0;

  // Camera intrinsics
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;

  // Target plane and safety
  double target_plane_z_ = 0.0;
  double min_valid_z_ = 0.05;
  double max_one_shot_ = 2.0;

  // Pixel-to-body mapping coefficients.
  // du_m = (u-cx)/fx * Z, positive means image right.
  // dv_m = (v-cy)/fy * Z, positive means image down.
  // body_x = bx_from_u * du_m + bx_from_v * dv_m
  // body_y = by_from_u * du_m + by_from_v * dv_m
  double bx_from_u_ = 0.0;
  double bx_from_v_ = -1.0;
  double by_from_u_ = -1.0;
  double by_from_v_ = 0.0;

  // Camera -> dropper offset in body frame, meters.
  // Meaning: dropper position relative to downward camera optical center.
  double dropper_offset_x_body_ = 0.0;
  double dropper_offset_y_body_ = 0.0;

  // Smoothing and timing
  int sample_num_ = 5;
  double detection_timeout_ = 0.5;
  double odom_timeout_ = 0.5;
  double publish_rate_ = 20.0;

  // State
  bool has_odom_ = false;
  nav_msgs::Odometry latest_odom_;
  ros::Time last_odom_stamp_;

  bool has_relative_ = false;
  ros::Time last_detection_stamp_;
  std::deque<geometry_msgs::Vector3> offset_samples_body_;

  template <typename T>
  void requireParam(const std::string& name, T& value) {
    if (!pnh_.getParam(name, value)) {
      ROS_FATAL_STREAM("[drop_target_relative] missing required private parameter: ~" << name);
      ros::shutdown();
    }
  }

  void loadParams() {
    requireParam("topics/detection", detection_topic_);
    requireParam("topics/odom", odom_topic_);
    requireParam("topics/relative", relative_topic_);

    requireParam("target/class_name", target_class_);
    requireParam("target/min_confidence", min_confidence_);

    requireParam("camera/fx", fx_);
    requireParam("camera/fy", fy_);
    requireParam("camera/cx", cx_);
    requireParam("camera/cy", cy_);

    requireParam("target_plane_z", target_plane_z_);
    requireParam("min_valid_z", min_valid_z_);
    requireParam("max_one_shot", max_one_shot_);

    requireParam("pixel_to_body/bx_from_u", bx_from_u_);
    requireParam("pixel_to_body/bx_from_v", bx_from_v_);
    requireParam("pixel_to_body/by_from_u", by_from_u_);
    requireParam("pixel_to_body/by_from_v", by_from_v_);

    requireParam("dropper/offset_x_body", dropper_offset_x_body_);
    requireParam("dropper/offset_y_body", dropper_offset_y_body_);

    requireParam("smoothing/sample_num", sample_num_);
    requireParam("timeout/detection", detection_timeout_);
    requireParam("timeout/odom", odom_timeout_);
    requireParam("publish_rate", publish_rate_);

    requireParam("output/frame", output_frame_);
    requireParam("frames/body", body_frame_id_);
    requireParam("frames/odom", odom_frame_id_);

    if (fx_ <= 0.0 || fy_ <= 0.0) {
      ROS_FATAL("[drop_target_relative] camera fx/fy must be positive.");
      ros::shutdown();
    }
    if (sample_num_ <= 0) {
      ROS_FATAL("[drop_target_relative] smoothing/sample_num must be > 0.");
      ros::shutdown();
    }
    if (publish_rate_ <= 0.0) {
      ROS_FATAL("[drop_target_relative] publish_rate must be > 0.");
      ros::shutdown();
    }
    if (output_frame_ != "body" && output_frame_ != "odom") {
      ROS_FATAL("[drop_target_relative] output/frame must be either 'body' or 'odom'.");
      ros::shutdown();
    }
  }

  static double clampAbs(double v, double max_abs) {
    if (max_abs <= 0.0) return v;
    return std::max(-max_abs, std::min(max_abs, v));
  }

  static double yawFromOdom(const nav_msgs::Odometry& odom) {
    const auto& q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
    has_odom_ = true;
    last_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  }

  void detectionCallback(const drop_target_relative::DetectionArray::ConstPtr& msg) {
    if (!has_odom_) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] received detection but no odom yet.");
      return;
    }

    const ros::Time now = ros::Time::now();
    if ((now - last_odom_stamp_).toSec() > odom_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] odom timeout, skip detection.");
      return;
    }

    const drop_target_relative::Detection* best = nullptr;
    double best_conf = -std::numeric_limits<double>::infinity();

    for (const auto& det : msg->detections) {
      if (det.class_name != target_class_) continue;
      if (det.confidence < min_confidence_) continue;
      if (det.width <= 0.0 || det.height <= 0.0) continue;
      if (det.confidence > best_conf) {
        best = &det;
        best_conf = det.confidence;
      }
    }

    if (!best) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] no valid target class '%s' in /yolo/detections.",
                        target_class_.c_str());
      return;
    }

    const double current_z = latest_odom_.pose.pose.position.z;
    const double Z = current_z - target_plane_z_;
    if (Z < min_valid_z_) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] invalid Z=%.3f. current_z=%.3f target_plane_z=%.3f",
                        Z, current_z, target_plane_z_);
      return;
    }

    const double u = best->x + best->width * 0.5;
    const double v = best->y + best->height * 0.5;

    const double du_m = ((u - cx_) / fx_) * Z;
    const double dv_m = ((v - cy_) / fy_) * Z;

    // Target relative to downward camera, expressed in UAV body horizontal axes.
    double target_from_camera_x_body = bx_from_u_ * du_m + bx_from_v_ * dv_m;
    double target_from_camera_y_body = by_from_u_ * du_m + by_from_v_ * dv_m;

    target_from_camera_x_body = clampAbs(target_from_camera_x_body, max_one_shot_);
    target_from_camera_y_body = clampAbs(target_from_camera_y_body, max_one_shot_);

    // Target relative to dropper outlet, expressed in UAV body horizontal axes.
    // If output is positive x, the target is in front of the dropper by that many meters.
    geometry_msgs::Vector3 offset_body;
    offset_body.x = target_from_camera_x_body - dropper_offset_x_body_;
    offset_body.y = target_from_camera_y_body - dropper_offset_y_body_;
    offset_body.z = 0.0;

    offset_samples_body_.push_back(offset_body);
    while (static_cast<int>(offset_samples_body_.size()) > sample_num_) {
      offset_samples_body_.pop_front();
    }

    has_relative_ = true;
    last_detection_stamp_ = msg->header.stamp.isZero() ? now : msg->header.stamp;

    ROS_INFO_THROTTLE(0.5,
      "[drop_target_relative] class=%s conf=%.2f center=(%.1f, %.1f) Z=%.2f body_offset=(%.3f, %.3f) samples=%zu/%d",
      best->class_name.c_str(), best->confidence, u, v, Z,
      offset_body.x, offset_body.y, offset_samples_body_.size(), sample_num_);
  }

  geometry_msgs::Vector3 averagedBodyOffset() const {
    geometry_msgs::Vector3 avg;
    avg.x = 0.0;
    avg.y = 0.0;
    avg.z = 0.0;

    if (offset_samples_body_.empty()) return avg;

    for (const auto& s : offset_samples_body_) {
      avg.x += s.x;
      avg.y += s.y;
      avg.z += s.z;
    }
    const double n = static_cast<double>(offset_samples_body_.size());
    avg.x /= n;
    avg.y /= n;
    avg.z /= n;
    return avg;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!has_relative_ || offset_samples_body_.empty()) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if ((now - last_detection_stamp_).toSec() > detection_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] detection timeout, stop publishing relative offset.");
      has_relative_ = false;
      offset_samples_body_.clear();
      return;
    }

    if (!has_odom_ || (now - last_odom_stamp_).toSec() > odom_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[drop_target_relative] odom timeout, stop publishing relative offset.");
      return;
    }

    const geometry_msgs::Vector3 avg_body = averagedBodyOffset();

    geometry_msgs::Vector3Stamped out;
    out.header.stamp = now;

    if (output_frame_ == "body") {
      out.header.frame_id = body_frame_id_;
      out.vector = avg_body;
    } else {
      const double yaw = yawFromOdom(latest_odom_);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      out.header.frame_id = odom_frame_id_;
      out.vector.x = c * avg_body.x - s * avg_body.y;
      out.vector.y = s * avg_body.x + c * avg_body.y;
      out.vector.z = 0.0;
    }

    rel_pub_.publish(out);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "drop_target_relative_node");
  DropTargetRelativeNode node;
  ros::spin();
  return 0;
}
