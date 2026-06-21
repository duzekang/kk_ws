#!/usr/bin/env python3
import math
import os
import yaml
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


def make_line(frame_id, ns, mid, rgba, scale=0.05):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


def load_points(path, traj_name, z_default):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise RuntimeError("YAML is empty")
    if traj_name not in data:
        raise RuntimeError("trajectory name '%s' not found; keys=%s" % (traj_name, list(data.keys())))
    traj = data[traj_name].get("trajectory", [])
    pts = []
    for item in traj:
        if isinstance(item, dict):
            x = float(item.get("x", 0.0))
            y = float(item.get("y", 0.0))
            z = float(item.get("z", z_default))
        else:
            x = float(item[0])
            y = float(item[1])
            z = float(item[2]) if len(item) >= 3 else z_default
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            pts.append((x, y, z))
    return pts


def publish_once(path_pub, marker_pub, pts, frame_id, wall_min_x, wall_min_y, enable_wall):
    now = rospy.Time.now()
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = now
    for x, y, z in pts:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    path_pub.publish(path)

    arr = MarkerArray()
    delete = Marker()
    delete.action = Marker.DELETEALL
    arr.markers.append(delete)

    line = make_line(frame_id, "yaml_path", 0, (0.0, 1.0, 0.1, 1.0), 0.065)
    for x, y, z in pts:
        p = Point(x=x, y=y, z=z + 0.05)
        line.points.append(p)
    arr.markers.append(line)

    sp = Marker()
    sp.header.frame_id = frame_id
    sp.header.stamp = now
    sp.ns = "yaml_points"
    sp.id = 1
    sp.type = Marker.SPHERE_LIST
    sp.action = Marker.ADD
    sp.pose.orientation.w = 1.0
    sp.scale.x = sp.scale.y = sp.scale.z = 0.075
    sp.color.r, sp.color.g, sp.color.b, sp.color.a = (1.0, 0.55, 0.0, 1.0)
    for x, y, z in pts:
        sp.points.append(Point(x=x, y=y, z=z + 0.08))
    arr.markers.append(sp)

    if pts:
        for mid, name, pt, color, scale in [
            (2, "start", pts[0], (0.0, 0.3, 1.0, 1.0), 0.16),
            (3, "goal", pts[-1], (1.0, 0.0, 0.0, 1.0), 0.16),
        ]:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = name
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pt[0]
            m.pose.position.y = pt[1]
            m.pose.position.z = pt[2] + 0.12
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            arr.markers.append(m)

    if enable_wall and pts:
        z = pts[0][2] + 0.12
        max_x = max([p[0] for p in pts] + [wall_min_x + 4.0]) + 1.0
        max_y = max([p[1] for p in pts] + [wall_min_y + 4.0]) + 1.0
        wx = make_line(frame_id, "wall_x_min", 4, (1.0, 0.0, 0.0, 1.0), 0.04)
        wx.points.append(Point(x=wall_min_x, y=wall_min_y, z=z))
        wx.points.append(Point(x=wall_min_x, y=max_y, z=z))
        arr.markers.append(wx)
        wy = make_line(frame_id, "wall_y_min", 5, (1.0, 0.0, 0.0, 1.0), 0.04)
        wy.points.append(Point(x=wall_min_x, y=wall_min_y, z=z))
        wy.points.append(Point(x=max_x, y=wall_min_y, z=z))
        arr.markers.append(wy)

    marker_pub.publish(arr)


def main():
    rospy.init_node("view_yaml_traj")
    yaml_file = rospy.get_param("~yaml_file", "/home/abot/kk_ws/src/radar_curve_planner/tmp/radar_generated_traj.yaml")
    traj_name = rospy.get_param("~traj_name", "traj1_4")
    frame_id = rospy.get_param("~frame_id", "camera_init")
    z_default = float(rospy.get_param("~z", 1.0))
    enable_wall = bool(rospy.get_param("~enable_virtual_wall", True))
    wall_min_x = float(rospy.get_param("~virtual_wall_min_x", -3.0))
    wall_min_y = float(rospy.get_param("~virtual_wall_min_y", -3.0))

    path_pub = rospy.Publisher("/radar_curve_planner/yaml_path", Path, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/radar_curve_planner/yaml_markers", MarkerArray, queue_size=1, latch=True)

    rate = rospy.Rate(2.0)
    last_mtime = None
    pts = []
    while not rospy.is_shutdown():
        try:
            mtime = os.path.getmtime(yaml_file)
            if mtime != last_mtime:
                pts = load_points(yaml_file, traj_name, z_default)
                last_mtime = mtime
                rospy.loginfo("[view_yaml_traj] loaded %d points from %s", len(pts), yaml_file)
            if pts:
                publish_once(path_pub, marker_pub, pts, frame_id, wall_min_x, wall_min_y, enable_wall)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[view_yaml_traj] %s", str(e))
        rate.sleep()


if __name__ == "__main__":
    main()
