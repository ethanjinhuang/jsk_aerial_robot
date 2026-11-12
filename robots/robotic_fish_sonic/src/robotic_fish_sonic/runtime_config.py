"""Latched effective settings; each node retains its own snapshot for late recorders."""
import json
import rospy
from robotic_fish_sonic.msg import RuntimeConfig


class ConfigRecorder:
    def __init__(self, device):
        self.device_id = device
        self.last_config_json = None
        self.config_publisher = rospy.Publisher("/robotic_fish/gain_control/config",
                                         RuntimeConfig, queue_size=1, latch=True)

    def publish(self, settings, **extra):
        config_json = json.dumps(dict(settings=settings, **extra), sort_keys=True, allow_nan=False)
        if config_json == self.last_config_json:
            return
        msg = RuntimeConfig()
        msg.timestamp = rospy.Time.now()
        msg.device = self.device_id
        msg.config_json = config_json
        self.config_publisher.publish(msg)
        self.last_config_json = config_json
