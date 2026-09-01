cmake_minimum_required(VERSION 3.0.2)
project(robotic_fish)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
        CATKIN_DEPENDS roscpp
)

catkin_install_python(PROGRAMS
        scripts/advance_teleop.py
        scripts/spring_teleop.py
        scripts/loop_send_servo_cmd.py
        scripts/servo_keyboard_cmd.py
        scripts/sonic_teleop.py
        scripts/teleop.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/loop_send_servo_cmd.bash
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
