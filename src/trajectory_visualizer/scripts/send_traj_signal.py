#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import rospy
from std_msgs.msg import Int32


def parse_code(s):
    s = str(s).strip().lower().replace("traj", "").replace("_", "").replace("-", "")
    aliases = {
        "13": 103,
        "14": 104,
        "23": 203,
        "24": 204,
        "103": 103,
        "104": 104,
        "203": 203,
        "204": 204,
    }
    return aliases.get(s, int(s))


def main():
    rospy.init_node("send_traj_signal")
    topic = rospy.get_param("~topic", "/trajectory_visualizer/route")
    raw = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param("~route", "104")
    code = parse_code(raw)
    pub = rospy.Publisher(topic, Int32, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(Int32(data=code))
    rospy.loginfo("[send_traj_signal] published route=%d to %s", code, topic)
    time.sleep(0.5)


if __name__ == "__main__":
    main()
