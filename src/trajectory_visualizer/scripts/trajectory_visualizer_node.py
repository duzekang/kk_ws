#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import re
import sys
import yaml

import rospy
from std_msgs.msg import String, Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryVisualizer:
    def __init__(self):
        self.ns = rospy.get_name()
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.yaml_file = rospy.get_param("~yaml_file", "")
        # Official external signal interface: std_msgs/Int32, data = 103/104/203/204.
        # 103 -> traj1_3, 104 -> traj1_4, 203 -> traj2_3, 204 -> traj2_4.
        self.route_topic = rospy.get_param("~route_topic", "/trajectory_visualizer/route")
        # Compatibility alias. You may set it to the same topic as route_topic or leave it empty to disable.
        self.select_id_topic = rospy.get_param("~select_id_topic", "/trajectory_visualizer/select_id")
        # Optional string interface for manual debugging only. Empty string disables it.
        self.select_topic = rospy.get_param("~select_topic", "")
        self.path_topic = rospy.get_param("~path_topic", "/radar_curve_planner/yaml_path")
        self.marker_topic = rospy.get_param("~marker_topic", "/radar_curve_planner/yaml_markers")
        self.extra_path_topics = rospy.get_param("~extra_path_topics", ["/competition_trajectory/path"])
        self.extra_marker_topics = rospy.get_param("~extra_marker_topics", ["/competition_trajectory/markers"])
        self.selected_type_topic = rospy.get_param("~selected_type_topic", "/trajectory_visualizer/selected_type")
        self.use_yaml_height = bool(rospy.get_param("~use_yaml_height", True))
        self.z_height = float(rospy.get_param("~z_height", 1.4))
        self.default_traj_name = rospy.get_param("~default_traj_name", "")
        self.latch = bool(rospy.get_param("~latch", True))
        self.republish_rate = float(rospy.get_param("~republish_rate", 1.0))
        self.line_width = float(rospy.get_param("~line_width", 0.06))
        self.point_size = float(rospy.get_param("~point_size", 0.12))
        self.start_goal_size = float(rospy.get_param("~start_goal_size", 0.22))
        self.text_size = float(rospy.get_param("~text_size", 0.22))

        if not self.yaml_file:
            self.yaml_file = self.default_yaml_path()

        self.data = self.load_yaml(self.yaml_file)
        if self.use_yaml_height and isinstance(self.data, dict) and "height" in self.data:
            try:
                self.z_height = float(self.data["height"])
            except Exception:
                rospy.logwarn("[trajectory_visualizer] yaml height is invalid, use z_height=%.3f", self.z_height)

        self.path_pubs = []
        self.marker_pubs = []
        self.add_path_pub(self.path_topic)
        for t in self.extra_path_topics:
            self.add_path_pub(t)
        self.add_marker_pub(self.marker_topic)
        for t in self.extra_marker_topics:
            self.add_marker_pub(t)

        self.selected_pub = rospy.Publisher(self.selected_type_topic, String, queue_size=1, latch=self.latch)

        self.last_path_msg = None
        self.last_marker_msg = None
        self.last_traj_name = ""

        self.subscribers = []
        if self.route_topic:
            self.subscribers.append(rospy.Subscriber(self.route_topic, Int32, self.on_route_code, queue_size=10))
        if self.select_id_topic and self.select_id_topic != self.route_topic:
            self.subscribers.append(rospy.Subscriber(self.select_id_topic, Int32, self.on_route_code, queue_size=10))
        if self.select_topic:
            self.subscribers.append(rospy.Subscriber(self.select_topic, String, self.on_string_select, queue_size=10))

        self.timer = None
        if self.republish_rate > 0.0:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.republish_rate), self.on_timer)

        keys = self.available_trajs()
        rospy.loginfo("[trajectory_visualizer] yaml=%s height=%.3f frame=%s", self.yaml_file, self.z_height, self.frame_id)
        rospy.loginfo("[trajectory_visualizer] available trajectories: %s", ", ".join(keys))
        rospy.loginfo("[trajectory_visualizer] listen Int32 route: %s | alias: %s | optional String: %s",
                      self.route_topic, self.select_id_topic, self.select_topic if self.select_topic else "disabled")
        rospy.loginfo("[trajectory_visualizer] route code mapping: 103->traj1_3, 104->traj1_4, 203->traj2_3, 204->traj2_4")
        rospy.loginfo("[trajectory_visualizer] publish Path: %s + %s", self.path_topic, self.extra_path_topics)
        rospy.loginfo("[trajectory_visualizer] publish MarkerArray: %s + %s", self.marker_topic, self.extra_marker_topics)

        if self.default_traj_name:
            self.publish_trajectory(self.default_traj_name)

    def default_yaml_path(self):
        # This fallback is only used when launch does not pass yaml_file.
        try:
            import rospkg
            return os.path.join(rospkg.RosPack().get_path("trajectory_visualizer"), "config", "lll.yaml")
        except Exception:
            return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "lll.yaml")

    def load_yaml(self, path):
        if not os.path.exists(path):
            rospy.logerr("[trajectory_visualizer] yaml file does not exist: %s", path)
            return {}
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            if data is None:
                data = {}
            if not isinstance(data, dict):
                rospy.logerr("[trajectory_visualizer] yaml root must be dict")
                return {}
            return data
        except Exception as e:
            rospy.logerr("[trajectory_visualizer] failed to read yaml %s: %s", path, e)
            return {}

    def add_path_pub(self, topic):
        if topic and topic not in [p.name for p in self.path_pubs]:
            self.path_pubs.append(rospy.Publisher(topic, Path, queue_size=1, latch=self.latch))

    def add_marker_pub(self, topic):
        if topic and topic not in [p.name for p in self.marker_pubs]:
            self.marker_pubs.append(rospy.Publisher(topic, MarkerArray, queue_size=1, latch=self.latch))

    def available_trajs(self):
        out = []
        for k, v in self.data.items():
            if isinstance(v, dict) and isinstance(v.get("trajectory", None), list):
                out.append(k)
        return sorted(out)

    def normalize_name(self, raw):
        if raw is None:
            return ""
        s = str(raw).strip()
        if not s:
            return ""

        # Accept YAML/JSON-like strings: "{type: traj1_4}" or "trajectory_type: traj1_4".
        if (":" in s and "traj" in s) or (s.startswith("{") and s.endswith("}")):
            try:
                obj = yaml.safe_load(s)
                if isinstance(obj, dict):
                    for key in ["trajectory_type", "traj_type", "traj_name", "type", "name", "trajectory"]:
                        if key in obj:
                            return self.normalize_name(obj[key])
            except Exception:
                pass

        s = s.strip().strip('"').strip("'")
        low = s.lower().replace(" ", "")
        low = low.replace("-", "_")
        low = low.replace("/", "_")

        if low in self.data and isinstance(self.data.get(low), dict):
            return low

        aliases = {}
        for a, b in [("13", "traj1_3"), ("23", "traj2_3"), ("14", "traj1_4"), ("24", "traj2_4")]:
            aliases[a] = b
            aliases["traj" + a] = b
            aliases[a[0] + "_" + a[1]] = b
            aliases[a[0] + "to" + a[1]] = b
            aliases["track" + a] = b
            aliases["track" + a[0] + "_" + a[1]] = b
            aliases["-" + a] = b + "_back"
            aliases[a + "back"] = b + "_back"
            aliases[a + "_back"] = b + "_back"
            aliases["back" + a] = b + "_back"
            aliases["traj" + a + "back"] = b + "_back"

        aliases.update({
            "103": "traj1_3",
            "104": "traj1_4",
            "203": "traj2_3",
            "204": "traj2_4",
        })

        if low in aliases:
            return aliases[low]

        # Convert traj13 -> traj1_3 etc.
        m = re.match(r"^traj([12])_?([34])(_back)?$", low)
        if m:
            name = "traj%s_%s%s" % (m.group(1), m.group(2), m.group(3) or "")
            return name

        return low

    def on_string_select(self, msg):
        name = self.normalize_name(msg.data)
        self.publish_trajectory(name)

    def on_route_code(self, msg):
        code = int(msg.data)
        # Official competition signal. Return routes use the same forward track, so no *_back mapping.
        mapping = {
            103: "traj1_3",
            104: "traj1_4",
            203: "traj2_3",
            204: "traj2_4",
            # Backward-compatible short codes for terminal debugging.
            13: "traj1_3",
            14: "traj1_4",
            23: "traj2_3",
            24: "traj2_4",
        }
        name = mapping.get(code, "")
        if not name:
            rospy.logwarn("[trajectory_visualizer] unsupported route code %d. Valid: 103, 104, 203, 204", code)
            return
        self.publish_trajectory(name)

    def get_points(self, name):
        if name not in self.data:
            return []
        item = self.data[name]
        if not isinstance(item, dict):
            return []
        traj = item.get("trajectory", [])
        pts = []
        for p in traj:
            if isinstance(p, (list, tuple)) and len(p) >= 2:
                try:
                    pts.append((float(p[0]), float(p[1])))
                except Exception:
                    pass
        return pts

    def publish_trajectory(self, name):
        name = self.normalize_name(name)
        pts = self.get_points(name)
        if len(pts) < 2:
            rospy.logwarn("[trajectory_visualizer] unknown or invalid trajectory '%s'. Available: %s", name, ", ".join(self.available_trajs()))
            return False

        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.frame_id

        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.z_height
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        markers = self.make_markers(name, pts, now)

        for pub in self.path_pubs:
            pub.publish(path)
        for pub in self.marker_pubs:
            pub.publish(markers)
        self.selected_pub.publish(String(data=name))

        self.last_path_msg = path
        self.last_marker_msg = markers
        self.last_traj_name = name
        rospy.loginfo("[trajectory_visualizer] published %s: points=%d height=%.2f path_topics=%d marker_topics=%d",
                      name, len(pts), self.z_height, len(self.path_pubs), len(self.marker_pubs))
        return True

    def make_markers(self, name, pts, stamp):
        arr = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = stamp
        delete_all.ns = "trajectory_visualizer"
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = "trajectory_visualizer"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = self.line_width
        line.color.r = 0.05
        line.color.g = 1.0
        line.color.b = 0.15
        line.color.a = 1.0
        for x, y in pts:
            line.points.append(Point(x=x, y=y, z=self.z_height))
        arr.markers.append(line)

        spheres = Marker()
        spheres.header.frame_id = self.frame_id
        spheres.header.stamp = stamp
        spheres.ns = "trajectory_visualizer"
        spheres.id = 2
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.pose.orientation.w = 1.0
        spheres.scale.x = self.point_size
        spheres.scale.y = self.point_size
        spheres.scale.z = self.point_size
        spheres.color.r = 1.0
        spheres.color.g = 0.55
        spheres.color.b = 0.05
        spheres.color.a = 1.0
        for x, y in pts:
            spheres.points.append(Point(x=x, y=y, z=self.z_height))
        arr.markers.append(spheres)

        arr.markers.append(self.make_sphere(3, pts[0], stamp, 0.0, 0.35, 1.0, "start"))
        arr.markers.append(self.make_sphere(4, pts[-1], stamp, 1.0, 0.05, 0.05, "goal"))
        arr.markers.append(self.make_text(5, pts[-1], stamp, name))

        return arr

    def make_sphere(self, marker_id, xy, stamp, r, g, b, ns_suffix):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "trajectory_visualizer_" + ns_suffix
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = xy[0]
        m.pose.position.y = xy[1]
        m.pose.position.z = self.z_height
        m.pose.orientation.w = 1.0
        m.scale.x = self.start_goal_size
        m.scale.y = self.start_goal_size
        m.scale.z = self.start_goal_size
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        return m

    def make_text(self, marker_id, xy, stamp, text):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "trajectory_visualizer_text"
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = xy[0]
        m.pose.position.y = xy[1]
        m.pose.position.z = self.z_height + 0.35
        m.pose.orientation.w = 1.0
        m.scale.z = self.text_size
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = text
        return m

    def on_timer(self, _event):
        if self.last_path_msg is None or self.last_marker_msg is None:
            return
        now = rospy.Time.now()
        self.last_path_msg.header.stamp = now
        for ps in self.last_path_msg.poses:
            ps.header.stamp = now
        for m in self.last_marker_msg.markers:
            m.header.stamp = now
        for pub in self.path_pubs:
            pub.publish(self.last_path_msg)
        for pub in self.marker_pubs:
            pub.publish(self.last_marker_msg)


def main():
    rospy.init_node("trajectory_visualizer_node")
    TrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
