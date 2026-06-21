#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>

struct Pt2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pt3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

static double dist2(const Pt2& a, const Pt2& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

static Pt2 operator+(const Pt2& a, const Pt2& b) { return Pt2{a.x + b.x, a.y + b.y}; }
static Pt2 operator-(const Pt2& a, const Pt2& b) { return Pt2{a.x - b.x, a.y - b.y}; }
static Pt2 operator*(const Pt2& a, double s) { return Pt2{a.x * s, a.y * s}; }

static double norm2(const Pt2& a) { return std::sqrt(a.x * a.x + a.y * a.y); }

static Pt2 normalize2(const Pt2& a) {
  const double n = norm2(a);
  if (n < 1e-9) return Pt2{0.0, 0.0};
  return Pt2{a.x / n, a.y / n};
}

class RadarMidpointNode {
public:
  RadarMidpointNode() : nh_(), pnh_("~") {
    loadParams();
    path_pub_ = nh_.advertise<nav_msgs::Path>(midpoints_topic_, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &RadarMidpointNode::cloudCb, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 1, &RadarMidpointNode::poseCb, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.2, max_plan_rate_)), &RadarMidpointNode::timerCb, this);
    ROS_INFO("[radar_midpoint] smooth EGO-LITE node started. cloud=%s pose=%s frame=%s", cloud_topic_.c_str(), pose_topic_.c_str(), frame_id_.c_str());
  }

private:
  struct Grid {
    double min_x = 0.0, min_y = 0.0, res = 0.1;
    int w = 0, h = 0;
    std::vector<uint8_t> occ;
    bool inBounds(int ix, int iy) const { return ix >= 0 && iy >= 0 && ix < w && iy < h; }
    int idx(int ix, int iy) const { return iy * w + ix; }
    Pt2 toWorld(int ix, int iy) const { return Pt2{min_x + (ix + 0.5) * res, min_y + (iy + 0.5) * res}; }
    bool toGrid(const Pt2& p, int& ix, int& iy) const {
      ix = static_cast<int>(std::floor((p.x - min_x) / res));
      iy = static_cast<int>(std::floor((p.y - min_y) / res));
      return inBounds(ix, iy);
    }
    bool isFree(int ix, int iy) const { return inBounds(ix, iy) && occ[idx(ix, iy)] == 0; }
  };

  void loadParams() {
    pnh_.param<std::string>("frame_id", frame_id_, "camera_init");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/mavros/local_position/pose");
    pnh_.param<std::string>("midpoints_topic", midpoints_topic_, "/radar_curve_planner/mid_points");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/radar_curve_planner/midpoint_markers");

    pnh_.param("target3_x", target_.x, -1.85);
    pnh_.param("target3_y", target_.y, -2.80);
    pnh_.param("target3_z", target_.z, 1.0);
    pnh_.param("use_target_z", use_target_z_, false);
    pnh_.param("flight_height", flight_height_, 1.0);

    pnh_.param("drone_diameter", drone_diameter_, 0.85);
    pnh_.param("safety_margin", safety_margin_, 0.04);
    pnh_.param("obstacle_inflation_extra", obstacle_inflation_extra_, 0.0);

    // 2D projection mode. By default we use a horizontal slice near the flight height:
    // z = projection_slice_center_z +/- projection_slice_half_width.
    // Points in this slice are compressed to XY obstacles, avoiding ground clutter while
    // still preventing the XY-only trajectory from crossing obstacles at flight height.
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

    pnh_.param("enable_virtual_wall", enable_virtual_wall_, true);
    pnh_.param("virtual_wall_min_x", wall_min_x_, -3.0);
    pnh_.param("virtual_wall_min_y", wall_min_y_, -3.0);
    pnh_.param("virtual_wall_max_x", wall_max_x_, 1e9);
    pnh_.param("virtual_wall_max_y", wall_max_y_, 1e9);
    pnh_.param("virtual_wall_margin", wall_margin_, 0.0);

    pnh_.param("grid_resolution", grid_res_, 0.10);
    pnh_.param("grid_padding", grid_padding_, 2.80);
    pnh_.param("max_grid_cells", max_grid_cells_, 250000);
    pnh_.param("max_search_time", max_search_time_, 0.35);
    pnh_.param("allow_diagonal", allow_diagonal_, true);

    pnh_.param("clear_start_radius", clear_start_radius_, 0.95);
    pnh_.param("clear_goal_radius", clear_goal_radius_, 0.25);

    pnh_.param("enable_line_of_sight_prune", enable_los_prune_, true);
    pnh_.param("enable_corner_rounding", enable_corner_rounding_, true);
    pnh_.param("corner_rounding_radius", corner_rounding_radius_, 0.35);
    pnh_.param("corner_rounding_samples", corner_rounding_samples_, 8);

    pnh_.param("enable_path_smoothing", enable_path_smoothing_, true);
    pnh_.param("smoothing_iterations", smoothing_iterations_, 45);
    pnh_.param("smoothing_alpha", smoothing_alpha_, 0.18);
    pnh_.param("smoothing_check_step", smoothing_check_step_, 0.04);

    pnh_.param("output_spacing", output_spacing_, 0.10);
    pnh_.param("min_output_points", min_output_points_, 6);
    pnh_.param("fallback_to_raw_if_smoothing_invalid", fallback_to_raw_if_smoothing_invalid_, true);

    pnh_.param("publish_markers", publish_markers_, true);
    pnh_.param("allow_plan_without_cloud", allow_plan_without_cloud_, true);
    pnh_.param("publish_straight_when_no_cloud", publish_straight_when_no_cloud_, true);
    pnh_.param("print_pose", print_pose_, true);
    pnh_.param("print_plan", print_plan_, true);
    pnh_.param("status_period", status_period_, 1.0);
    pnh_.param("max_plan_rate", max_plan_rate_, 5.0);
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    latest_cloud_ = msg;
    has_cloud_ = true;
    cloud_total_ = static_cast<int>(msg->width * msg->height);
  }

  void poseCb(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_.x = msg->pose.position.x;
    current_.y = msg->pose.position.y;
    current_.z = msg->pose.position.z;
    has_pose_ = true;
  }

  bool insideWall(const Pt2& p) const {
    if (!enable_virtual_wall_) return true;
    return p.x >= wall_min_x_ + wall_margin_ && p.y >= wall_min_y_ + wall_margin_ &&
           p.x <= wall_max_x_ - wall_margin_ && p.y <= wall_max_y_ - wall_margin_;
  }

  std::vector<Pt2> filteredCloud2D(const Pt2& start) const {
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
          // Height-slice 2D compression: obstacle points near the selected slice height
          // block the XY trajectory; low ground clutter outside the slice is ignored.
          if (z < projection_min_z_ || z > projection_max_z_) continue;
        } else if (use_2d_z_filter_ && (z < obstacle_min_z_ || z > obstacle_max_z_)) {
          continue;
        }
        const Pt2 p{x, y};
        if (dist2(p, start) < self_filter_radius_) continue;
        pts.push_back(p);
      }
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "[radar_midpoint] PointCloud2 iterator failed: %s", e.what());
    }
    return pts;
  }

  Grid buildGrid(const Pt2& start, const Pt2& goal, const std::vector<Pt2>& obs, std::string& reason) const {
    Grid g;
    g.res = std::max(0.03, grid_res_);
    double min_x = std::min(start.x, goal.x) - grid_padding_;
    double max_x = std::max(start.x, goal.x) + grid_padding_;
    double min_y = std::min(start.y, goal.y) - grid_padding_;
    double max_y = std::max(start.y, goal.y) + grid_padding_;

    if (enable_virtual_wall_) {
      min_x = std::max(min_x, wall_min_x_ + wall_margin_);
      min_y = std::max(min_y, wall_min_y_ + wall_margin_);
      max_x = std::min(max_x, wall_max_x_ - wall_margin_);
      max_y = std::min(max_y, wall_max_y_ - wall_margin_);
    }

    if (max_x <= min_x || max_y <= min_y) {
      reason = "invalid_grid_bounds";
      return g;
    }

    g.min_x = min_x;
    g.min_y = min_y;
    g.w = static_cast<int>(std::ceil((max_x - min_x) / g.res));
    g.h = static_cast<int>(std::ceil((max_y - min_y) / g.res));
    const int64_t cells = static_cast<int64_t>(g.w) * static_cast<int64_t>(g.h);
    if (g.w <= 1 || g.h <= 1 || cells > max_grid_cells_) {
      reason = "grid_too_large";
      g.w = 0;
      g.h = 0;
      return g;
    }
    g.occ.assign(static_cast<size_t>(cells), 0);

    const double inflation = drone_diameter_ * 0.5 + safety_margin_ + obstacle_inflation_extra_;
    const int r_cells = static_cast<int>(std::ceil(inflation / g.res));
    for (const Pt2& p : obs) {
      if (p.x < min_x - inflation || p.x > max_x + inflation || p.y < min_y - inflation || p.y > max_y + inflation) continue;
      int cx = 0, cy = 0;
      if (!g.toGrid(p, cx, cy)) continue;
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          const int ix = cx + dx;
          const int iy = cy + dy;
          if (!g.inBounds(ix, iy)) continue;
          const Pt2 wp = g.toWorld(ix, iy);
          if (dist2(wp, p) <= inflation) g.occ[g.idx(ix, iy)] = 1;
        }
      }
    }

    clearCircle(g, start, clear_start_radius_);
    clearCircle(g, goal, clear_goal_radius_);
    reason = "ok";
    return g;
  }

  void clearCircle(Grid& g, const Pt2& c, double radius) const {
    int cx = 0, cy = 0;
    if (!g.toGrid(c, cx, cy)) return;
    const int r = static_cast<int>(std::ceil(radius / g.res));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int ix = cx + dx;
        const int iy = cy + dy;
        if (!g.inBounds(ix, iy)) continue;
        if (dist2(g.toWorld(ix, iy), c) <= radius) g.occ[g.idx(ix, iy)] = 0;
      }
    }
  }

  bool segmentFree(const Grid& g, const Pt2& a, const Pt2& b, double step) const {
    if (!insideWall(a) || !insideWall(b)) return false;
    const double len = dist2(a, b);
    const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(0.02, step))));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      const Pt2 p{a.x * (1.0 - t) + b.x * t, a.y * (1.0 - t) + b.y * t};
      if (!insideWall(p)) return false;
      int ix = 0, iy = 0;
      if (!g.toGrid(p, ix, iy) || !g.isFree(ix, iy)) return false;
    }
    return true;
  }

  std::vector<Pt2> astar(const Grid& g, const Pt2& start, const Pt2& goal, std::string& reason) const {
    int sx = 0, sy = 0, gx = 0, gy = 0;
    if (!g.toGrid(start, sx, sy) || !g.toGrid(goal, gx, gy)) {
      reason = "start_or_goal_outside_grid";
      return {};
    }
    if (!g.isFree(sx, sy) || !g.isFree(gx, gy)) {
      reason = "start_or_goal_occupied";
      return {};
    }

    const int n = g.w * g.h;
    std::vector<double> cost(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);

    struct QN { int idx; double f; };
    struct Cmp { bool operator()(const QN& a, const QN& b) const { return a.f > b.f; } };
    std::priority_queue<QN, std::vector<QN>, Cmp> pq;

    const int sidx = g.idx(sx, sy);
    const int gidx = g.idx(gx, gy);
    auto hfun = [&](int ix, int iy) {
      const double dx = static_cast<double>(ix - gx);
      const double dy = static_cast<double>(iy - gy);
      return std::sqrt(dx * dx + dy * dy);
    };

    cost[sidx] = 0.0;
    pq.push(QN{sidx, hfun(sx, sy)});
    const ros::WallTime start_time = ros::WallTime::now();
    int expand = 0;

    const int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    const int dirs8[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const int (*dirs)[2] = allow_diagonal_ ? dirs8 : dirs4;
    const int ndirs = allow_diagonal_ ? 8 : 4;

    while (!pq.empty()) {
      if ((ros::WallTime::now() - start_time).toSec() > max_search_time_) {
        reason = "astar_timeout";
        return {};
      }
      const QN cur = pq.top(); pq.pop();
      if (closed[cur.idx]) continue;
      closed[cur.idx] = 1;
      ++expand;
      if (cur.idx == gidx) break;
      const int cx = cur.idx % g.w;
      const int cy = cur.idx / g.w;
      for (int k = 0; k < ndirs; ++k) {
        const int nx = cx + dirs[k][0];
        const int ny = cy + dirs[k][1];
        if (!g.isFree(nx, ny)) continue;
        if (std::abs(dirs[k][0]) + std::abs(dirs[k][1]) == 2) {
          if (!g.isFree(cx + dirs[k][0], cy) || !g.isFree(cx, cy + dirs[k][1])) continue;
        }
        const int ni = g.idx(nx, ny);
        if (closed[ni]) continue;
        const double step = (std::abs(dirs[k][0]) + std::abs(dirs[k][1]) == 2) ? std::sqrt(2.0) : 1.0;
        const double nc = cost[cur.idx] + step;
        if (nc < cost[ni]) {
          cost[ni] = nc;
          parent[ni] = cur.idx;
          pq.push(QN{ni, nc + hfun(nx, ny)});
        }
      }
    }

    if (parent[gidx] < 0 && gidx != sidx) {
      reason = "no_free_astar_path";
      return {};
    }

    std::vector<Pt2> rev;
    int cur = gidx;
    rev.push_back(goal);
    while (cur != sidx && cur >= 0) {
      const int ix = cur % g.w;
      const int iy = cur / g.w;
      rev.push_back(g.toWorld(ix, iy));
      cur = parent[cur];
    }
    rev.push_back(start);
    std::reverse(rev.begin(), rev.end());
    reason = "ok";
    return rev;
  }

  std::vector<Pt2> lineOfSightPrune(const Grid& g, const std::vector<Pt2>& path) const {
    if (path.size() <= 2 || !enable_los_prune_) return path;
    std::vector<Pt2> out;
    size_t i = 0;
    out.push_back(path.front());
    while (i + 1 < path.size()) {
      size_t best = i + 1;
      for (size_t j = path.size() - 1; j > i + 1; --j) {
        if (segmentFree(g, path[i], path[j], grid_res_ * 0.5)) {
          best = j;
          break;
        }
      }
      out.push_back(path[best]);
      i = best;
    }
    return out;
  }

  std::vector<Pt2> cornerRound(const Grid& g, const std::vector<Pt2>& path) const {
    if (!enable_corner_rounding_ || path.size() <= 2) return path;
    std::vector<Pt2> out;
    out.push_back(path.front());
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      const Pt2 A = path[i - 1];
      const Pt2 B = path[i];
      const Pt2 C = path[i + 1];
      const double l1 = dist2(A, B);
      const double l2 = dist2(B, C);
      double base_r = std::min(corner_rounding_radius_, 0.45 * std::min(l1, l2));
      bool accepted = false;
      if (base_r > 0.05) {
        const Pt2 dir_to_A = normalize2(A - B);
        const Pt2 dir_to_C = normalize2(C - B);
        for (double factor : {1.0, 0.7, 0.45, 0.25}) {
          const double r = base_r * factor;
          const Pt2 p0 = B + dir_to_A * r;
          const Pt2 p2 = B + dir_to_C * r;
          std::vector<Pt2> candidate;
          candidate.push_back(p0);
          const int samples = std::max(3, corner_rounding_samples_);
          for (int s = 1; s < samples; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(samples);
            const double u = 1.0 - t;
            Pt2 q;
            q.x = u * u * p0.x + 2.0 * u * t * B.x + t * t * p2.x;
            q.y = u * u * p0.y + 2.0 * u * t * B.y + t * t * p2.y;
            candidate.push_back(q);
          }
          candidate.push_back(p2);
          bool ok = true;
          Pt2 prev = out.back();
          for (const Pt2& q : candidate) {
            if (!segmentFree(g, prev, q, smoothing_check_step_)) { ok = false; break; }
            prev = q;
          }
          if (ok && !segmentFree(g, prev, C, smoothing_check_step_)) ok = false;
          if (ok) {
            for (const Pt2& q : candidate) out.push_back(q);
            accepted = true;
            break;
          }
        }
      }
      if (!accepted) out.push_back(B);
    }
    out.push_back(path.back());
    return removeNearDuplicates(out, 0.02);
  }

  std::vector<Pt2> iterativeSmooth(const Grid& g, const std::vector<Pt2>& path) const {
    if (!enable_path_smoothing_ || path.size() <= 2) return path;
    std::vector<Pt2> p = path;
    const double alpha = std::max(0.0, std::min(0.45, smoothing_alpha_));
    for (int it = 0; it < smoothing_iterations_; ++it) {
      for (size_t i = 1; i + 1 < p.size(); ++i) {
        const Pt2 avg{0.5 * (p[i - 1].x + p[i + 1].x), 0.5 * (p[i - 1].y + p[i + 1].y)};
        const Pt2 cand{p[i].x * (1.0 - alpha) + avg.x * alpha, p[i].y * (1.0 - alpha) + avg.y * alpha};
        if (!insideWall(cand)) continue;
        if (segmentFree(g, p[i - 1], cand, smoothing_check_step_) && segmentFree(g, cand, p[i + 1], smoothing_check_step_)) {
          p[i] = cand;
        }
      }
    }
    return removeNearDuplicates(p, 0.02);
  }

  std::vector<Pt2> removeNearDuplicates(const std::vector<Pt2>& in, double eps) const {
    std::vector<Pt2> out;
    for (const Pt2& p : in) {
      if (out.empty() || dist2(out.back(), p) >= eps) out.push_back(p);
    }
    return out;
  }

  bool pathValidOnGrid(const Grid& g, const std::vector<Pt2>& path) const {
    if (path.size() < 2) return false;
    for (size_t i = 1; i < path.size(); ++i) {
      if (!segmentFree(g, path[i - 1], path[i], smoothing_check_step_)) return false;
    }
    return true;
  }

  std::vector<Pt2> resamplePath(const std::vector<Pt2>& path, double spacing) const {
    if (path.size() <= 1) return path;
    spacing = std::max(0.03, spacing);
    std::vector<Pt2> out;
    out.push_back(path.front());
    double carry = 0.0;
    Pt2 last = path.front();
    for (size_t i = 1; i < path.size(); ++i) {
      Pt2 a = last;
      const Pt2 b = path[i];
      double seg = dist2(a, b);
      if (seg < 1e-6) continue;
      Pt2 dir = normalize2(b - a);
      while (carry + seg >= spacing) {
        const double need = spacing - carry;
        Pt2 np{a.x + dir.x * need, a.y + dir.y * need};
        out.push_back(np);
        a = np;
        seg = dist2(a, b);
        carry = 0.0;
        dir = normalize2(b - a);
      }
      carry += seg;
      last = b;
    }
    if (dist2(out.back(), path.back()) > 0.02) out.push_back(path.back());
    return out;
  }

  double pathLength(const std::vector<Pt2>& p) const {
    double L = 0.0;
    for (size_t i = 1; i < p.size(); ++i) L += dist2(p[i - 1], p[i]);
    return L;
  }

  double minClearance(const std::vector<Pt2>& path, const std::vector<Pt2>& obs) const {
    if (path.empty() || obs.empty()) return 999.0;
    double best = 999.0;
    for (const Pt2& p : path) {
      for (const Pt2& q : obs) best = std::min(best, dist2(p, q));
    }
    return best;
  }

  void publishPath(const std::vector<Pt2>& path, double z) const {
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    for (const Pt2& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = z;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_.publish(msg);
  }

  visualization_msgs::Marker makeLineMarker(int id, const std::string& ns, float r, float g, float b, float a, double scale) const {
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

  void publishMarkers(const std::vector<Pt2>& raw, const std::vector<Pt2>& smooth, double z) const {
    if (!publish_markers_) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::Marker raw_line = makeLineMarker(0, "astar_raw", 0.7f, 0.7f, 0.7f, 0.55f, 0.025);
    for (const Pt2& p : raw) {
      geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = z + 0.02; raw_line.points.push_back(gp);
    }
    arr.markers.push_back(raw_line);

    visualization_msgs::Marker sm_line = makeLineMarker(1, "smooth_path", 0.0f, 0.9f, 1.0f, 1.0f, 0.055);
    for (const Pt2& p : smooth) {
      geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = z + 0.05; sm_line.points.push_back(gp);
    }
    arr.markers.push_back(sm_line);

    visualization_msgs::Marker pts;
    pts.header.frame_id = frame_id_;
    pts.header.stamp = ros::Time::now();
    pts.ns = "smooth_points";
    pts.id = 2;
    pts.type = visualization_msgs::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.08; pts.scale.y = 0.08; pts.scale.z = 0.08;
    pts.color.r = 1.0f; pts.color.g = 0.55f; pts.color.b = 0.0f; pts.color.a = 1.0f;
    for (const Pt2& p : smooth) {
      geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = z + 0.08; pts.points.push_back(gp);
    }
    arr.markers.push_back(pts);

    if (enable_virtual_wall_) {
      visualization_msgs::Marker wx = makeLineMarker(3, "virtual_wall_xmin", 1.0f, 0.0f, 0.0f, 1.0f, 0.04);
      geometry_msgs::Point a, b;
      a.x = wall_min_x_; a.y = wall_min_y_; a.z = z + 0.12;
      b.x = wall_min_x_; b.y = std::min(wall_max_y_, target_.y + grid_padding_); b.z = z + 0.12;
      if (b.y < a.y + 0.1) b.y = a.y + 4.0;
      wx.points.push_back(a); wx.points.push_back(b);
      arr.markers.push_back(wx);
      visualization_msgs::Marker wy = makeLineMarker(4, "virtual_wall_ymin", 1.0f, 0.0f, 0.0f, 1.0f, 0.04);
      a.x = wall_min_x_; a.y = wall_min_y_; a.z = z + 0.12;
      b.x = std::min(wall_max_x_, target_.x + grid_padding_); b.y = wall_min_y_; b.z = z + 0.12;
      if (b.x < a.x + 0.1) b.x = a.x + 4.0;
      wy.points.push_back(a); wy.points.push_back(b);
      arr.markers.push_back(wy);
    }

    marker_pub_.publish(arr);
  }

  void timerCb(const ros::TimerEvent&) {
    if (!has_pose_) return;
    const ros::Time now = ros::Time::now();
    if ((now - last_status_time_).toSec() >= status_period_ && print_pose_) {
      ROS_INFO("[radar_midpoint] current_pose: x=%.3f y=%.3f z=%.3f has_cloud=%d cloud_points=%d", current_.x, current_.y, current_.z, has_cloud_ ? 1 : 0, cloud_total_);
      last_status_time_ = now;
    }

    Pt2 start{current_.x, current_.y};
    Pt2 goal{target_.x, target_.y};
    const double z = use_target_z_ ? target_.z : flight_height_;
    const double safe_radius = drone_diameter_ * 0.5 + safety_margin_ + obstacle_inflation_extra_;

    if (!has_cloud_ && allow_plan_without_cloud_ && publish_straight_when_no_cloud_) {
      std::vector<Pt2> straight{start, goal};
      auto out = resamplePath(straight, output_spacing_);
      publishPath(out, z);
      publishMarkers(straight, out, z);
      ROS_WARN_THROTTLE(1.0, "[radar_midpoint] no cloud, publishing straight fallback");
      return;
    }
    if (!has_cloud_) return;

    const std::vector<Pt2> obs = filteredCloud2D(start);
    std::string reason;
    Grid grid = buildGrid(start, goal, obs, reason);
    if (grid.w <= 0 || grid.h <= 0) {
      ROS_WARN_THROTTLE(1.0, "[radar_midpoint] EGO-LITE-SMOOTH plan: success=0 reason=%s", reason.c_str());
      return;
    }

    std::vector<Pt2> raw = astar(grid, start, goal, reason);
    if (raw.empty()) {
      ROS_WARN_THROTTLE(1.0, "[radar_midpoint] EGO-LITE-SMOOTH-2D plan: success=0 reason=%s cloud_total=%d projected_obs=%zu grid=%dx%d safe_radius=%.2f projection=%s[%.2f,%.2f]",
                        reason.c_str(), cloud_total_, obs.size(), grid.w, grid.h, safe_radius,
                        project_pointcloud_to_2d_ ? (use_height_slice_projection_ ? "slice2d" : "column2d") : "height_band",
                        project_pointcloud_to_2d_ ? projection_min_z_ : obstacle_min_z_,
                        project_pointcloud_to_2d_ ? projection_max_z_ : obstacle_max_z_);
      return;
    }

    std::vector<Pt2> pruned = lineOfSightPrune(grid, raw);
    std::vector<Pt2> rounded = cornerRound(grid, pruned);
    std::vector<Pt2> smoothed = iterativeSmooth(grid, rounded);
    if (!pathValidOnGrid(grid, smoothed)) {
      if (fallback_to_raw_if_smoothing_invalid_) {
        smoothed = pruned;
        reason = "smoothing_invalid_fallback_to_pruned";
      } else {
        ROS_WARN_THROTTLE(1.0, "[radar_midpoint] EGO-LITE-SMOOTH plan: success=0 reason=smoothing_invalid");
        return;
      }
    }
    std::vector<Pt2> out = resamplePath(smoothed, output_spacing_);
    if (static_cast<int>(out.size()) < min_output_points_ && out.size() >= 2) {
      out = resamplePath(smoothed, std::max(0.03, pathLength(smoothed) / std::max(1, min_output_points_ - 1)));
    }

    publishPath(out, z);
    publishMarkers(raw, out, z);

    if (print_plan_) {
      ROS_INFO_THROTTLE(0.2,
        "[radar_midpoint] EGO-LITE-SMOOTH-2D plan: success=1 reason=%s cloud_total=%d projected_obs=%zu grid=%dx%d raw=%zu pruned=%zu smooth=%zu out=%zu len=%.2f min_clear=%.2f safe_radius=%.2f projection=%s[%.2f,%.2f] wall=x>=%.2f,y>=%.2f rounding=%d smoothing=%d",
        reason.c_str(), cloud_total_, obs.size(), grid.w, grid.h, raw.size(), pruned.size(), smoothed.size(), out.size(), pathLength(out), minClearance(out, obs), safe_radius,
        project_pointcloud_to_2d_ ? (use_height_slice_projection_ ? "slice2d" : "column2d") : "height_band",
        project_pointcloud_to_2d_ ? projection_min_z_ : obstacle_min_z_,
        project_pointcloud_to_2d_ ? projection_max_z_ : obstacle_max_z_,
        wall_min_x_, wall_min_y_, enable_corner_rounding_ ? 1 : 0, enable_path_smoothing_ ? 1 : 0);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher path_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;

  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  bool has_cloud_ = false;
  bool has_pose_ = false;
  int cloud_total_ = 0;
  Pt3 current_;
  Pt3 target_;

  std::string frame_id_, cloud_topic_, pose_topic_, midpoints_topic_, marker_topic_;
  bool use_target_z_ = false;
  double flight_height_ = 1.0;
  double drone_diameter_ = 0.85, safety_margin_ = 0.04, obstacle_inflation_extra_ = 0.0;
  bool project_pointcloud_to_2d_ = true;
  bool use_height_slice_projection_ = true;
  double projection_slice_center_z_ = 0.80, projection_slice_half_width_ = 0.15;
  double projection_min_z_ = 0.65, projection_max_z_ = 0.95;
  bool use_2d_z_filter_ = true;
  double obstacle_min_z_ = 0.60, obstacle_max_z_ = 2.00, self_filter_radius_ = 0.90;
  bool enable_virtual_wall_ = true;
  double wall_min_x_ = -3.0, wall_min_y_ = -3.0, wall_max_x_ = 1e9, wall_max_y_ = 1e9, wall_margin_ = 0.0;
  double grid_res_ = 0.10, grid_padding_ = 2.80, max_search_time_ = 0.35;
  int max_grid_cells_ = 250000;
  bool allow_diagonal_ = true;
  double clear_start_radius_ = 0.95, clear_goal_radius_ = 0.25;
  bool enable_los_prune_ = true, enable_corner_rounding_ = true, enable_path_smoothing_ = true;
  double corner_rounding_radius_ = 0.35;
  int corner_rounding_samples_ = 8;
  int smoothing_iterations_ = 45;
  double smoothing_alpha_ = 0.18, smoothing_check_step_ = 0.04;
  double output_spacing_ = 0.10;
  int min_output_points_ = 6;
  bool fallback_to_raw_if_smoothing_invalid_ = true;
  bool publish_markers_ = true, allow_plan_without_cloud_ = true, publish_straight_when_no_cloud_ = true;
  bool print_pose_ = true, print_plan_ = true;
  double status_period_ = 1.0, max_plan_rate_ = 5.0;
  ros::Time last_status_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_midpoint_node");
  RadarMidpointNode node;
  ros::spin();
  return 0;
}
