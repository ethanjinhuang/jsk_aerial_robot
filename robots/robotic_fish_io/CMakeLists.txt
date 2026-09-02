cmake_minimum_required(VERSION 3.0.2)
project(robotic_fish_io)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

add_message_files(
  FILES
  AdcSample.msg
  AdcSampleArray.msg
  GainControlState.msg
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
  CATKIN_DEPENDS diagnostic_msgs message_runtime rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/agc_node.py
  scripts/adc_node.py
  scripts/dac_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_drivers.py)
  catkin_add_nosetests(test/test_calibration.py)
  catkin_add_nosetests(test/test_agc.py)
endif()
