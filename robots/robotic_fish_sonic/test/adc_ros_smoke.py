#!/usr/bin/env python3
"""Real ADC ROS transport + record-all readback with fake I2C, no hardware.

The ADC node and its driver run normally except that open_adc returns a
deterministic in-memory bus. No DAC node, serial bridge or actuator is started.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'test'))


def wait_until(predicate, description, timeout_s=8.):
    deadline_monotonic_s = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline_monotonic_s:
            raise RuntimeError('Timed out: ' + description)
        time.sleep(.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-script', type=Path, default=ROOT / 'scripts/adc_node.py')
    args = parser.parse_args()
    port = 11430
    try:
        probe = socket.create_connection(('127.0.0.1', port), timeout=.2)
    except OSError:
        pass
    else:
        probe.close()
        raise RuntimeError('test master port 11430 is already in use')
    output = Path(tempfile.mkdtemp(prefix='fish-adc-ros-', dir='/tmp'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1',
                      ROS_HOME=str(output), ROS_LOG_DIR=str(output / 'log'))
    os.environ.pop('ROS_HOSTNAME', None)
    master_log = (output / 'master.log').open('w')
    recorder_log = (output / 'recorder.log').open('w')
    master = recorder = worker = None
    try:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log, stderr=subprocess.STDOUT)

        def ready():
            if master.poll() is not None:
                raise RuntimeError('ROS master exited; see ' + str(output))
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/adc_smoke')
                return True
            except OSError:
                return False

        wait_until(ready, 'isolated ROS master')
        import rospy
        import rosbag
        from robotic_fish_sonic.msg import AdcSampleArray, DacState, RuntimeConfig
        from test_drivers import FakeAdcBus

        class SimulatedAdcBus(FakeAdcBus):
            def close(self):
                pass
        rospy.init_node('adc_publication_smoke', disable_signals=True)
        rospy.set_param('~adc/sample_group_rate_hz', 20.)
        rospy.set_param('~adc/calibration/enabled', True)
        rospy.set_param('~adc/calibration/file', str(ROOT / 'config/calibration/'
                         'adc_independent_transfer_2026-09-27-13-51-52.json'))
        rospy.set_param('~adc/publish_array', False)  # Legacy setting must not suppress the only output.
        spec = importlib.util.spec_from_file_location('adc_under_test', args.node_script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.adc_driver.open_adc = lambda _: SimulatedAdcBus(
            raw_by_channel={0: 8000, 1: 16000, 2: 24000})
        node = module.AdcNode()
        received = []
        subscriber = rospy.Subscriber('/robotic_fish/adc/samples', AdcSampleArray, received.append)
        dac_publisher = rospy.Publisher('/robotic_fish/dac/state', DacState, queue_size=1, latch=True)
        wait_until(lambda: dac_publisher.get_num_connections() > 0, 'DAC-state subscriber')
        dac_message = DacState(timestamp=rospy.Time.now(), dac_volt_target=.8, dac_volt=.8,
                               status_dac_feedback=True)
        dac_publisher.publish(dac_message)
        wait_until(lambda: node.dac_voltage_state == (True, .8), 'confirmed DAC snapshot')

        errors = []

        def run_adc():
            try:
                node.run()
            except rospy.ROSInterruptException:
                pass
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=run_adc, daemon=True)
        worker.start()
        wait_until(lambda: len(received) >= 3 or bool(errors), 'first ADC groups')
        assert not errors, errors
        bag_path = output / 'record_all.bag'
        # Start after ADC/config publication to test the user's late-recording workflow.
        recorder = subprocess.Popen(['rosbag', 'record', '-a', '-O', str(bag_path)],
                                     stdout=recorder_log, stderr=subprocess.STDOUT)
        wait_until(lambda: node.samples_publisher.get_num_connections() >= 2,
                   'record-all ADC subscription')
        first_count = len(received)
        wait_until(lambda: len(received) >= first_count + 25, '25 recorded sample groups')
        published_topics = dict(rospy.get_published_topics('/robotic_fish/adc'))
        assert published_topics == {'/robotic_fish/adc/samples': 'robotic_fish_sonic/AdcSampleArray'}, published_topics
        recorder.send_signal(signal.SIGINT)
        assert recorder.wait(timeout=10) == 0
        with rosbag.Bag(str(bag_path), 'r') as bag:
            topics = bag.get_type_and_topic_info().topics
            assert '/robotic_fish/adc/sample' not in topics
            groups = [m for _, m, _ in bag.read_messages(topics=['/robotic_fish/adc/samples'])]
            configs = [m for _, m, _ in bag.read_messages(topics=['/robotic_fish/gain_control/config'])]
            assert len(groups) >= 20, len(groups)
            # rosbag creates classes from embedded definitions; class identity
            # need not match the locally generated RuntimeConfig class.
            assert configs and all(m._type == RuntimeConfig._type
                                   and m._md5sum == RuntimeConfig._md5sum for m in configs)
            config = json.loads(next(m.config_json for m in configs if m.device == 'adc'))
            assert config['settings']['publish_array'] is True
            assert 'sample' not in config['topics']
            assert config['calibration']['calibration_id']
            for group in groups:
                assert [s.channel_id for s in group.samples] == [0, 1, 2]
                assert [s.volt_raw for s in group.samples] == [1., 2., 3.]
                assert all(s.status_cali and s.status_dac_feedback and s.dac_volt == .8
                           for s in group.samples)
            assert any(all(s.mean_1s_ready for s in group.samples) for group in groups)
        print('PASS: ADC real ROS transport, record-all contains groups only, legacy message fields, '
              'calibration, DAC tagging, one-second means and late latched config; ' + str(bag_path), flush=True)
        subscriber.unregister()
        dac_publisher.unregister()
    finally:
        if 'rospy' in locals():
            rospy.signal_shutdown('smoke test complete')
        if worker is not None:
            worker.join(timeout=3.)
            if worker.is_alive():
                raise RuntimeError('ADC worker did not stop')
        for process in (recorder, master):
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        master_log.close()
        recorder_log.close()


if __name__ == '__main__':
    main()
