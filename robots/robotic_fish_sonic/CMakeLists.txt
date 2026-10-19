cmake_minimum_required(VERSION 3.0.2)
project(robotic_fish_sonic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

add_message_files(
  FILES
  AdcSample.msg
  AdcSampleArray.msg
  DacState.msg
  RuntimeConfig.msg
  GainControlState.msg
  ChannelComparison.msg
  QuadrantProbabilities.msg
)

add_service_files(
  FILES
  SetDacVoltage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs message_runtime rospy sensor_msgs std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/agc_node.py
  scripts/adc_node.py
  scripts/dac_node.py
  scripts/calibrate_from_bag.py
  scripts/quadrant_node.py
  scripts/replay_quadrants.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md VALIDATION.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_drivers.py)
  catkin_add_nosetests(test/test_adc_mean.py)
  catkin_add_nosetests(test/test_calibration.py)
  catkin_add_nosetests(test/test_bag_calibration.py)
  catkin_add_nosetests(test/test_agc.py)
  catkin_add_nosetests(test/test_manual_gain.py)
  catkin_add_nosetests(test/test_safety_window.py)
  catkin_add_nosetests(test/test_node_safety.py)
  catkin_add_nosetests(test/test_acoustic_core.py)
endif()
