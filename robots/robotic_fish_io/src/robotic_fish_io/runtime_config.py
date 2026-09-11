"""Latched effective settings; each node retains its own snapshot for late recorders."""
import json
import rospy
from robotic_fish_io.msg import RuntimeConfig


class ConfigRecorder:
    def __init__(self, device):
        self.device = device
        self.last_payload = None
        self.publisher = rospy.Publisher("/robotic_fish/gain_control/config",
                                         RuntimeConfig, queue_size=1, latch=True)

    def publish(self, settings, **extra):
        payload = json.dumps(dict(settings=settings, **extra), sort_keys=True, allow_nan=False)
        if payload == self.last_payload:
            return
        msg = RuntimeConfig()
        msg.timestamp = rospy.Time.now()
        msg.device = self.device
        msg.config_json = payload
        self.publisher.publish(msg)
        self.last_payload = payload
