cmake_minimum_required(VERSION 3.0.2)
project(araig_benchmarks)

find_package(catkin REQUIRED COMPONENTS
  araig_msgs
  rospy
  marvelmind_nav
  std_msgs
  geometry_msgs
  araig_test_runners
)

catkin_package(
   CATKIN_DEPENDS rospy araig_msgs marvelmind_nav std_msgs geometry_msgs araig_test_runners
)

catkin_install_python(PROGRAMS
   scripts/bench_braking.py
   scripts/mock_marvel.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()