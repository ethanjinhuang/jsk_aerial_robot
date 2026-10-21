#!/usr/bin/env python3
"""Print Share snapshots on the machine running this node, without hardware IO."""
import threading

import rospy
from std_msgs.msg import String

from robotic_fish_sonic.status_display import COLOR_MODES, write_snapshot


class StatusConsole:
    def __init__(self):
        self.color = rospy.get_param('~color', 'auto')
        if self.color not in COLOR_MODES:
            raise ValueError('color must be auto, always or never')
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber('/robotic_fish/sonic/status_snapshot', String,
                                    self.callback, queue_size=10)
        rospy.on_shutdown(self.sub.unregister)
        rospy.loginfo('Share status console ready (color=%s)', self.color)

    def callback(self, message):
        with self.lock:
            write_snapshot(message.data, self.color)


if __name__ == '__main__':
    rospy.init_node('sonic_status_console')
    try:
        node = StatusConsole()
        rospy.spin()
    except ValueError as exc:
        rospy.logfatal('Invalid status console configuration: %s', exc)
        raise SystemExit(1)
