# CMake generated Testfile for 
# Source directory: /home/usanz/colcon_ws/src/navigation_l4ros2/bt_behavior/tests
# Build directory: /home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(bt_action_test "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/test_results/bt_behavior/bt_action_test.gtest.xml" "--package-name" "bt_behavior" "--output-file" "/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/ament_cmake_gtest/bt_action_test.txt" "--command" "/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/tests/bt_action_test" "--gtest_output=xml:/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/test_results/bt_behavior/bt_action_test.gtest.xml")
set_tests_properties(bt_action_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/tests/bt_action_test" TIMEOUT "60" WORKING_DIRECTORY "/home/usanz/colcon_ws/src/navigation_l4ros2/build/bt_behavior/tests" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/usanz/colcon_ws/src/navigation_l4ros2/bt_behavior/tests/CMakeLists.txt;2;ament_add_gtest;/home/usanz/colcon_ws/src/navigation_l4ros2/bt_behavior/tests/CMakeLists.txt;0;")
subdirs("../gtest")
