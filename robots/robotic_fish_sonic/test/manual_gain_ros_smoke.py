#!/usr/bin/env python3
"""Real gain-node timers and ROS services with a fake DAC; no hardware access."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def wait_for(predicate, description, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-script', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'scripts/agc_node.py')
    args = parser.parse_args()
    port = 11430
    try:
        connection = socket.create_connection(('127.0.0.1', port), timeout=.2)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError('isolated master port 11430 is already in use')
    output = Path(tempfile.mkdtemp(prefix='fish-manual-gain-ros-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11430', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output), ROS_LOG_DIR=str(output / 'log'))
    with (output / 'master.log').open('w') as log:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=subprocess.STDOUT)
        node = None
        try:
            def ready():
                if master.poll() is not None:
                    raise RuntimeError('isolated master failed; logs: ' + str(output))
                try:
                    xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/manual_smoke')
                    return True
                except OSError:
                    return False
            wait_for(ready, 'master startup', 10.)
            import rospy
            from sensor_msgs.msg import Joy
            from std_srvs.srv import SetBool
            from robotic_fish_sonic.msg import AdcSample, AdcSampleArray
            from robotic_fish_sonic.srv import SetDacVoltage, SetDacVoltageResponse
            rospy.init_node('manual_gain_smoke', disable_signals=True)
            prefix = '/manual_gain_test'
            values = dict(mode='manual', manual_voltage=.1, manual_ramp_step_v=.02,
                          manual_ramp_interval_s=.1, start_voltage=0.,
                          adc_samples_topic=prefix+'/adc', dac_state_topic=prefix+'/dac',
                          set_voltage_service=prefix+'/set', joy_topic=prefix+'/joy',
                          enable_service=prefix+'/enable', reset_safety_service=prefix+'/reset',
                          state_topic=prefix+'/state')
            for key, value in values.items():
                rospy.set_param('~gain_control/'+key, value)
            commands = []
            def apply(request):
                assert request.channel == 1 and math.isfinite(request.voltage)
                assert 0. <= request.voltage <= 5.
                commands.append(round(request.voltage, 2))
                return SetDacVoltageResponse(True, request.voltage, 'simulated DAC')
            service = rospy.Service(prefix+'/set', SetDacVoltage, apply)
            spec = importlib.util.spec_from_file_location('gain_under_test', args.node_script)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            node = module.GainControlNode()
            joy_pub = rospy.Publisher(prefix+'/joy', Joy, queue_size=1)
            adc_pub = rospy.Publisher(prefix+'/adc', AdcSampleArray, queue_size=1)
            wait_for(lambda: joy_pub.get_num_connections() and adc_pub.get_num_connections(), 'connections')

            def at(voltage):
                return commands and abs(commands[-1]-voltage) < .001

            def press_up():
                for axis in (0., 1., 0.):
                    msg = Joy(axes=[0.]*10+[axis])
                    joy_pub.publish(msg)
                    time.sleep(.06)

            def adc(voltages, stale=False):
                stamp = rospy.Time.now() - rospy.Duration(5. if stale else .01)
                samples = [AdcSample(channel_id=i, timestamp=stamp, volt_raw=v,
                                     volt_cali=v, adc_code=0)
                           for i, v in enumerate(voltages)]
                adc_pub.publish(AdcSampleArray(samples=samples))
                time.sleep(.1)

            wait_for(lambda: at(.1), 'manual initialization and ramp without ADC')
            assert commands[0] == 0. and not node._adc_is_valid()
            press_up()
            wait_for(lambda: at(.11), 'manual Joy without ADC')
            adc([-.005, -.006, -.007])
            assert not node._adc_is_valid() and not node.last_error
            press_up()
            wait_for(lambda: at(.12), 'negative ADC must not block manual DAC')
            assert node._state_message().state != 'adc_stale'
            print('PASS: real timer initializes and ramps DAC with no ADC; Joy still works with negative ADC', flush=True)

            adc([-.005, 4.05, .1])
            wait_for(lambda: node.controller.safety_active and at(.02), 'overrange overrides invalid companion')
            before = len(commands)
            press_up()
            adc([.1]*3, stale=True)
            time.sleep(.15)
            assert node.controller.safety_active and len(commands) == before
            adc([.1]*3)
            wait_for(lambda: not node.controller.safety_active, 'fresh ADC releases protection')
            assert node.controller.manual_limited
            time.sleep(.2)
            assert len(commands) == before, 'old manual target resumed after protection'
            press_up()
            wait_for(lambda: at(.03), 'new manual press after protection')
            time.sleep(1.1)
            assert not node._adc_is_valid()
            press_up()
            wait_for(lambda: at(.04), 'stale ADC must not block manual DAC')
            print('PASS: raw overrange still reduces DAC; stale ADC cannot clear protection; new presses resume independently', flush=True)

            enable = rospy.ServiceProxy(prefix+'/enable', SetBool)
            assert enable(True).success
            before = len(commands)
            press_up()
            adc([float('nan'), -.005, .1])
            time.sleep(.2)
            assert node.controller.mode == 'closed_loop' and len(commands) == before
            assert not node._adc_is_valid()
            assert enable(False).success
            time.sleep(.1)
            assert len(commands) == before
            print('PASS: closed loop still requires valid ADC; off mode does not run manual commands', flush=True)
            node._shutdown()
            node.diagnostic_timer.shutdown()
            rospy.signal_shutdown('manual gain smoke complete')
            service.shutdown()
            print('PASS: clean shutdown; isolated logs: ' + str(output), flush=True)
        finally:
            if node is not None:
                node._shutdown()
                node.diagnostic_timer.shutdown()
            master.send_signal(signal.SIGINT)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait(timeout=2)


if __name__ == '__main__':
    main()
