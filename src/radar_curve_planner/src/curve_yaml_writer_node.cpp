#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

struct Pt2 { double x = 0.0; double y = 0.0; };
struct Pt3 { double x = 0.0; double y = 0.0; double z = 0.0; };

static double dist2(const Pt2& a, const Pt2& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}
static double dist3xy(const Pt3& a, const Pt3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}
static Pt2 normalize2(const Pt2& v) {
  const double n = std::sqrt(v.x * v.x + v.y * v.y);
  if (n < 1e-9) return Pt2{0.0, 0.0};
  return Pt2{v.x / n, v.y / n};
}

class CurveYamlWriterNode {
public:
  CurveYamlWriterNode() : nh_(), pnh_("~") {
    loadParams();
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CurveYamlWriterNode::cloudCb, this);
    path_sub_ = nh_.subscribe(midpoints_topic_, 1, &CurveYamlWriterNode::pathCb, this);
    fitted_path_pub_ = nh_.advertise<nav_msgs::Path>(fitted_path_topic_, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>(path_valid_topic_, 1, true);
    ROS_INFO("[curve_writer] smooth writer started. input=%s output=%s mode=%s", midpoints_topic_.c_str(), output_yaml_.c_str(), curve_mode_.c_str());
  }

private:
  void loadParams() {
    pnh_.param<std::string>("frame_id", frame_id_, "camera_init");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("midpoints_topic", midpoints_topic_, "/radar_curve_planner/mid_points");
    pnh_.param<std::string>("fitted_path_topic", fitted_path_topic_, "/radar_curve_planner/fitted_path");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/radar_curve_planner/curve_markers");
    pnh_.param<std::string>("path_valid_topic", path_valid_topic_, "/radar_curve_planner/path_valid");
    pnh_.param<std::string>("output_yaml", output_yaml_, "/home/abot/kk_ws/src/radar_curve_planner/tmp/radar_generated_traj.yaml");
    pnh_.param<std::string>("traj_name", traj_name_, "traj1_4");
    pnh_.param<std::string>("output_coordinate_mode", output_coordinate_mode_, "absolute");
    pnh_.param<std::string>("curve_mode", curve_mode_, "resample");

    pnh_.param("flight_height", flight_height_, 1.0);
    pnh_.param("drone_diameter", drone_diameter_, 0.85);
    pnh_.param("safety_margin", safety_margin_, 0.04);

    // Must match radar_midpoint_node: validate YAML using the same 2D height-slice projection.
    pnh_.param("project_pointcloud_to_2d", project_pointcloud_to_2d_, true);
    pnh_.param("use_height_slice_projection", use_height_slice_projection_, true);
    pnh_.param("projection_slice_center_z", projection_slice_center_z_, 0.80);
    pnh_.param("projection_slice_half_width", projection_slice_half_width_, 0.15);
    pnh_.param("projection_min_z", projection_min_z_, 0.65);
    pnh_.param("projection_max_z", projection_max_z_, 0.95);
    if (project_pointcloud_to_2d_ && use_height_slice_projection_) {
      const double hw = std::fabs(projection_slice_half_width_);
      projection_min_z_ = projection_slice_center_z_ - hw;
      projection_max_z_ = projection_slice_center_z_ + hw;
    }

    // Backward-compatible height-band mode. Used only when project_pointcloud_to_2d=false.
    pnh_.param("use_2d_z_filter", use_2d_z_filter_, true);
    pnh_.param("obstacle_min_z", obstacle_min_z_, 0.60);
    pnh_.param("obstacle_max_z", obstacle_max_z_, 2.00);
    pnh_.param("self_filter_radius", self_filter_radius_, 0.90);
    pnh_.param("start_ignore_curve_distance", start_ignore_curve_distance_, 0.80);

    pnh_.param("enable_virtual_wall", enable_virtual_wall_, true);
    pnh_.param("virtual_wall_min_x", wall_min_x_, -3.0);
    pnh_.param("virtual_wall_min_y", wall_min_y_, -3.0);
    pnh_.param("virtual_wall_max_x", wall_max_x_, 1e9);
    pnh_.param("virtual_wall_max_y", wall_max_y_, 1e9);
    pnh_.param("virtual_wall_margin", wall_margin_, 0.0);

    pnh_.param("min_collision_points", min_collision_points_, 24);
    pnh_.param("resample_step", resample_step_, 0.08);
    pnh_.param("max_curvature", max_curvature_, 999.0);
    pnh_.param("max_turn_angle_deg", max_turn_angle_deg_, 179.0);
    pnh_.param("min_segment_len", min_segment_len_, 0.02);
    pnh_.param("sample_step", sample_step_, 0.06);
    pnh_.param("write_even_if_invalid", write_even_if_invalid_, false);
    pnh_.param("one_shot", one_shot_, false);
    pnh_.param("one_shot_require_valid", one_shot_require_valid_, true);
    pnh_.param("print_collision_cloud_count", print_collision_cloud_count_, true);
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    latest_cloud_ = msg;
    has_cloud_ = true;
  }

  bool insideWall(const Pt3& p) const {
    if (!enable_virtual_wall_) return true;
    return p.x >= wall_min_x_ + wall_margin_ && p.y >= wall_min_y_ + wall_margin_ &&
           p.x <= wall_max_x_ - wall_margin_ && p.y <= wall_max_y_ - wall_margin_;
  }

  std::vector<Pt2> cloud2D(const Pt3& start) const {
    std::vector<Pt2> pts;
    if (!latest_cloud_) return pts;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (project_pointcloud_to_2d_) {
          // Validate with the same height-slice 2D projection as the planner.
          // The final trajectory uses XY, so points in this slice block the footprint.
          if (z < projection_min_z_ || z > projection_max_z_) continue;
        } else if (use_2d_z_filter_ && (z < obstacle_min_z_ || z > obstacle_max_z_)) {
          continue;
        }
        const double dx = x - start.x;
        const double dy = y - start.y;
        if (std::sqrt(dx * dx + dy * dy) < self_filter_radius_) continue;
        pts.push_back(Pt2{x, y});
      }
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "[curve_writer] PointCloud2 iterator failed: %s", e.what());
    }
    return pts;
  }

  std::vector<Pt3> resample3(const std::vector<Pt3>& path, double step) const {
    if (path.size() <= 1) return path;
    step = std::max(0.03, step);
    std::vector<Pt3> out;
    out.push_back(path.front());
    double carry = 0.0;
    Pt3 last = path.front();
    for (size_t i = 1; i < path.size(); ++i) {
      Pt3 a = last;
      const Pt3 b = path[i];
      double seg = dist3xy(a, b);
      if (seg < 1e-6) continue;
      while (carry + seg >= step) {
        const double need = step - carry;
        const double t = need / seg;
        Pt3 np;
        np.x = a.x + (b.x - a.x) * t;
        np.y = a.y + (b.y - a.y) * t;
        np.z = a.z + (b.z - a.z) * t;
        out.push_back(np);
        a = np;
        seg = dist3xy(a, b);
        carry = 0.0;
      }
      carry += seg;
      last = b;
    }
    if (dist3xy(out.back(), path.back()) > 0.02) out.push_back(path.back());
    return out;
  }

  std::vector<Pt3> pathFromMsg(const nav_msgs::PathConstPtr& msg) const {
    std::vector<Pt3> pts;
    for (const auto& ps : msg->poses) {
      Pt3 p;
      p.x = ps.pose.position.x;
      p.y = ps.pose.position.y;
      p.z = ps.pose.position.z;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (pts.empty() || dist3xy(pts.back(), p) >= min_segment_len_) pts.push_back(p);
    }
    return pts;
  }

  double pathLength(const std::vector<Pt3>& path) const {
    double L = 0.0;
    for (size_t i = 1; i < path.size(); ++i) L += dist3xy(path[i - 1], path[i]);
    return L;
  }

  bool validatePath(const std::vector<Pt3>& path, std::string& reason, int& near_points_out, double& max_curv_out, double& max_angle_out, bool& wall_bad_out, double& wall_v_out, int& collision_index_out, double& collision_s_out) const {
    reason = "ok";
    near_points_out = 0;
    max_curv_out = 0.0;
    max_angle_out = 0.0;
    wall_bad_out = false;
    wall_v_out = 0.0;
    collision_index_out = -1;
    collision_s_out = 0.0;

    if (path.size() < 2) {
      reason = "too_few_points";
      return false;
    }

    for (const Pt3& p : path) {
      if (!insideWall(p)) {
        wall_bad_out = true;
        double v = 0.0;
        if (p.x < wall_min_x_ + wall_margin_) v = std::max(v, wall_min_x_ + wall_margin_ - p.x);
        if (p.y < wall_min_y_ + wall_margin_) v = std::max(v, wall_min_y_ + wall_margin_ - p.y);
        if (p.x > wall_max_x_ - wall_margin_) v = std::max(v, p.x - (wall_max_x_ - wall_margin_));
        if (p.y > wall_max_y_ - wall_margin_) v = std::max(v, p.y - (wall_max_y_ - wall_margin_));
        wall_v_out = std::max(wall_v_out, v);
      }
    }
    if (wall_bad_out) {
      reason = "virtual_wall_violation";
      return false;
    }

    for (size_t i = 1; i + 1 < path.size(); ++i) {
      const Pt2 a{path[i].x - path[i - 1].x, path[i].y - path[i - 1].y};
      const Pt2 b{path[i + 1].x - path[i].x, path[i + 1].y - path[i].y};
      const double la = std::sqrt(a.x * a.x + a.y * a.y);
      const double lb = std::sqrt(b.x * b.x + b.y * b.y);
      if (la < 1e-6 || lb < 1e-6) continue;
      double c = (a.x * b.x + a.y * b.y) / (la * lb);
      c = std::max(-1.0, std::min(1.0, c));
      const double angle = std::acos(c);
      const double angle_deg = angle * 180.0 / M_PI;
      max_angle_out = std::max(max_angle_out, angle_deg);
      const double curv = angle / std::max(0.5 * (la + lb), 1e-3);
      max_curv_out = std::max(max_curv_out, curv);
    }
    if (max_angle_out > max_turn_angle_deg_) {
      reason = "turn_angle_too_large";
      return false;
    }
    if (max_curv_out > max_curvature_) {
      reason = "curvature_too_large";
      return false;
    }

    const std::vector<Pt2> obs = cloud2D(path.front());
    const double collision_radius = drone_diameter_ * 0.5 + safety_margin_;
    double s_acc = 0.0;
    int sample_index = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      const Pt3 a = path[i - 1];
      const Pt3 b = path[i];
      const double seg = dist3xy(a, b);
      const int n = std::max(1, static_cast<int>(std::ceil(seg / std::max(0.02, sample_step_))));
      for (int j = 0; j <= n; ++j) {
        const double t = static_cast<double>(j) / static_cast<double>(n);
        Pt3 p;
        p.x = a.x * (1.0 - t) + b.x * t;
        p.y = a.y * (1.0 - t) + b.y * t;
        p.z = a.z * (1.0 - t) + b.z * t;
        const double s_here = s_acc + seg * t;
        if (s_here < start_ignore_curve_distance_) {
          ++sample_index;
          continue;
        }
        int near = 0;
        for (const Pt2& q : obs) {
          const double dx = p.x - q.x;
          const double dy = p.y - q.y;
          if (dx * dx + dy * dy <= collision_radius * collision_radius) ++near;
        }
        if (near >= min_collision_points_) {
          near_points_out = near;
          collision_index_out = sample_index;
          collision_s_out = s_here;
          std::ostringstream oss;
          oss << "collision at curve index " << collision_index_out << ", s=" << std::fixed << std::setprecision(2) << collision_s_out << "m, near points=" << near;
          reason = oss.str();
          return false;
        }
        near_points_out = std::max(near_points_out, near);
        ++sample_index;
      }
      s_acc += seg;
    }
    return true;
  }

  bool ensureParentDir(const std::string& file) const {
    const size_t pos = file.find_last_of('/');
    if (pos == std::string::npos) return true;
    std::string dir = file.substr(0, pos);
    if (dir.empty()) return true;
    std::string cur;
    if (dir[0] == '/') cur = "/";
    std::stringstream ss(dir);
    std::string item;
    while (std::getline(ss, item, '/')) {
      if (item.empty()) continue;
      if (!cur.empty() && cur.back() != '/') cur += "/";
      cur += item;
      if (::mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    return true;
  }

  bool writeYaml(const std::vector<Pt3>& path) const {
    if (!ensureParentDir(output_yaml_)) return false;
    std::ofstream out(output_yaml_.c_str());
    if (!out.is_open()) return false;
    const Pt3 start = path.front();
    out << std::fixed << std::setprecision(6);
    out << traj_name_ << ":\n";
    out << "  trajectory:\n";
    for (const Pt3& p : path) {
      double x = p.x;
      double y = p.y;
      double z = p.z;
      if (output_coordinate_mode_ == "relative_to_start") {
        x -= start.x;
        y -= start.y;
        z -= start.z;
      }
      out << "    - [" << x << ", " << y << ", " << z << "]\n";
    }
    return true;
  }

  void publishFittedPath(const std::vector<Pt3>& path) const {
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    for (const Pt3& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = p.z;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    fitted_path_pub_.publish(msg);
  }

  visualization_msgs::Marker lineMarker(int id, const std::string& ns, float r, float g, float b, float a, double scale) const {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  void publishMarkers(const std::vector<Pt3>& path, bool valid) const {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::Marker line = lineMarker(0, "final_curve", valid ? 0.0f : 1.0f, valid ? 1.0f : 0.0f, 0.0f, 1.0f, 0.065);
    for (const Pt3& p : path) {
      geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = p.z + 0.05; line.points.push_back(gp);
    }
    arr.markers.push_back(line);

    visualization_msgs::Marker pts;
    pts.header.frame_id = frame_id_;
    pts.header.stamp = ros::Time::now();
    pts.ns = "final_points";
    pts.id = 1;
    pts.type = visualization_msgs::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.07; pts.scale.y = 0.07; pts.scale.z = 0.07;
    pts.color.r = 1.0f; pts.color.g = 0.55f; pts.color.b = 0.0f; pts.color.a = 1.0f;
    for (const Pt3& p : path) {
      geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = p.z + 0.08; pts.points.push_back(gp);
    }
    arr.markers.push_back(pts);

    if (enable_virtual_wall_ && !path.empty()) {
      const double z = path.front().z + 0.12;
      const double maxx = std::max(path.front().x, path.back().x) + 2.0;
      const double maxy = std::max(path.front().y, path.back().y) + 2.0;
      visualization_msgs::Marker wx = lineMarker(2, "wall_xmin", 1.0f, 0.0f, 0.0f, 1.0f, 0.04);
      geometry_msgs::Point a, b;
      a.x = wall_min_x_; a.y = wall_min_y_; a.z = z;
      b.x = wall_min_x_; b.y = maxy; b.z = z;
      wx.points.push_back(a); wx.points.push_back(b);
      arr.markers.push_back(wx);
      visualization_msgs::Marker wy = lineMarker(3, "wall_ymin", 1.0f, 0.0f, 0.0f, 1.0f, 0.04);
      a.x = wall_min_x_; a.y = wall_min_y_; a.z = z;
      b.x = maxx; b.y = wall_min_y_; b.z = z;
      wy.points.push_back(a); wy.points.push_back(b);
      arr.markers.push_back(wy);
    }
    marker_pub_.publish(arr);
  }

  void pathCb(const nav_msgs::PathConstPtr& msg) {
    std::vector<Pt3> path = pathFromMsg(msg);
    if (curve_mode_ == "resample" || curve_mode_ == "polyline") path = resample3(path, resample_step_);
    if (path.empty()) return;

    std::string reason;
    int near_points = 0;
    double max_curv = 0.0, max_angle = 0.0, wall_v = 0.0, collision_s = 0.0;
    bool wall_bad = false;
    int collision_idx = -1;
    const bool valid = validatePath(path, reason, near_points, max_curv, max_angle, wall_bad, wall_v, collision_idx, collision_s);

    publishFittedPath(path);
    publishMarkers(path, valid);
    std_msgs::Bool vb;
    vb.data = valid;
    valid_pub_.publish(vb);

    if (valid || write_even_if_invalid_) {
      if (writeYaml(path)) {
        ROS_INFO("[curve_writer] yaml written: %s", output_yaml_.c_str());
      } else {
        ROS_ERROR("[curve_writer] failed to write yaml: %s", output_yaml_.c_str());
      }
    } else {
      ROS_WARN("[curve_writer] invalid path, yaml not written. Set write_even_if_invalid=true for debug output.");
    }

    const double collision_radius = drone_diameter_ * 0.5 + safety_margin_;
    ROS_INFO("[curve_writer] curve=%zu mode=%s length=%.2fm valid=%d reason=%s max_curv=%.3f max_angle=%.1fdeg collision_radius=%.2f near_points=%d min_collision_points=%d self_filter=%.2f start_ignore_s=%.2f wall_bad=%d wall_v=%.3f cloud_pts=%zu projection=%s[%.2f,%.2f]",
             path.size(), curve_mode_.c_str(), pathLength(path), valid ? 1 : 0, reason.c_str(), max_curv, max_angle,
             collision_radius, near_points, min_collision_points_, self_filter_radius_, start_ignore_curve_distance_,
             wall_bad ? 1 : 0, wall_v, latest_cloud_ ? static_cast<size_t>(latest_cloud_->width * latest_cloud_->height) : 0,
             project_pointcloud_to_2d_ ? (use_height_slice_projection_ ? "slice2d" : "column2d") : "height_band",
             project_pointcloud_to_2d_ ? projection_min_z_ : obstacle_min_z_,
             project_pointcloud_to_2d_ ? projection_max_z_ : obstacle_max_z_);

    if (one_shot_ && (valid || !one_shot_require_valid_)) ros::shutdown();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher fitted_path_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher valid_pub_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  bool has_cloud_ = false;

  std::string frame_id_, cloud_topic_, midpoints_topic_, fitted_path_topic_, marker_topic_, path_valid_topic_;
  std::string output_yaml_, traj_name_, output_coordinate_mode_, curve_mode_;
  double flight_height_ = 1.0, drone_diameter_ = 0.85, safety_margin_ = 0.04;
  bool project_pointcloud_to_2d_ = true;
  bool use_height_slice_projection_ = true;
  double projection_slice_center_z_ = 0.80, projection_slice_half_width_ = 0.15;
  double projection_min_z_ = 0.65, projection_max_z_ = 0.95;
  bool use_2d_z_filter_ = true;
  double obstacle_min_z_ = 0.60, obstacle_max_z_ = 2.00;
  double self_filter_radius_ = 0.90, start_ignore_curve_distance_ = 0.80;
  bool enable_virtual_wall_ = true;
  double wall_min_x_ = -3.0, wall_min_y_ = -3.0, wall_max_x_ = 1e9, wall_max_y_ = 1e9, wall_margin_ = 0.0;
  int min_collision_points_ = 24;
  double resample_step_ = 0.08, max_curvature_ = 999.0, max_turn_angle_deg_ = 179.0, min_segment_len_ = 0.02, sample_step_ = 0.06;
  bool write_even_if_invalid_ = false, one_shot_ = false, one_shot_require_valid_ = true, print_collision_cloud_count_ = true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "curve_yaml_writer_node");
  CurveYamlWriterNode node;
  ros::spin();
  return 0;
}
