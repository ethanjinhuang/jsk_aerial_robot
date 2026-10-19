#!/usr/bin/env python3
"""Publish results even when quality flags are present; never refresh held stamps."""
import threading
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robotic_fish_sonic.msg import AdcSampleArray
from robotic_fish_sonic.msg import QuadrantProbabilities, ChannelComparison
from robotic_fish_sonic.acoustic_core import CLASSES, Estimator, Model, Status


class Node:
    def __init__(self):
        path = rospy.get_param('~model', '')
        self.engine = Estimator(Model(path) if path else None,
                                rospy.get_param('~window_s', .5),
                                rospy.get_param('~gap_s', .2),
                                rospy.get_param('~low_signal_v', .001))
        self.lock = threading.Lock()
        self.pub = rospy.Publisher('~probabilities', QuadrantProbabilities, queue_size=20)
        self.comparison_pub = rospy.Publisher('~comparison', ChannelComparison, queue_size=20)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=2)
        self.sub = rospy.Subscriber('/robotic_fish/adc/samples', AdcSampleArray, self.receive, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick, reset=True)
        if not path:
            rospy.loginfo('Model-free channel comparison active; direction probabilities unavailable without a trained model.')

    def publish(self, result):
        msg = QuadrantProbabilities()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'fish_base_link'
        for key in ('estimate_stamp', 'window_start', 'window_end'):
            setattr(msg, key, rospy.Time.from_sec(result[key] or 0.))
        probabilities = result['probabilities'] or [float('nan')] * 4
        for name, value in zip(CLASSES, probabilities):
            setattr(msg, 'probability_' + name, value)
        msg.probability_left = probabilities[2] + probabilities[3]
        msg.probability_right = probabilities[0] + probabilities[1]
        for key in ('has_estimate', 'updated', 'status', 'estimate_status', 'valid_sample_count', 'model_id', 'timestamp_semantics'):
            setattr(msg, key, result[key])
        self.pub.publish(msg)
        comparison = result['comparison']
        msg = ChannelComparison()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'fish_base_link'
        for key in ('has_comparison', 'updated', 'reliable', 'status'):
            setattr(msg, key, comparison[key])
        if comparison['has_comparison']:
            msg.sample_stamp = rospy.Time.from_sec(comparison['sample_stamp'])
            for key in ('left_right_normalized_difference', 'head_sides_normalized_difference'):
                setattr(msg, key, comparison[key])
        self.comparison_pub.publish(msg)

    def receive(self, msg):
        with self.lock:
            self.publish(self.engine.push(msg, rospy.Time.now().to_sec()))

    def tick(self, event):
        with self.lock:
            result = self.engine.snapshot(rospy.Time.now().to_sec())
            self.publish(result)
            diagnostic = DiagnosticStatus()
            diagnostic.name = 'robotic_fish_sonic/quadrants'
            diagnostic.hardware_id = 'adc0_adc1_adc2'
            flags = [flag.name for flag in Status if result['comparison']['status'] & flag]
            diagnostic.level = DiagnosticStatus.WARN if flags else DiagnosticStatus.OK
            diagnostic.message = ', '.join(flags) if flags else 'Channel comparison active'
            if result['status'] & Status.MODEL_UNAVAILABLE:
                diagnostic.message += '; MODEL_UNAVAILABLE: comparison only, direction probabilities unavailable'
            diagnostic.values = [KeyValue(key, str(result[key])) for key in ('model_id', 'has_estimate')]
            diagnostic.values += [KeyValue(key, str(result['comparison'].get(key, 0))) for key in
                                  ('has_comparison', 'reliable', 'status')]
            array = DiagnosticArray()
            array.header.stamp = rospy.Time.now()
            array.status = [diagnostic]
            self.diagnostics_pub.publish(array)


if __name__ == '__main__':
    rospy.init_node('acoustic_quadrants')
    Node()
    rospy.spin()
