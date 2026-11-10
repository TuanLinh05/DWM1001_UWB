cmake_minimum_required(VERSION 3.8)
project(uwb_eskf)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(uwb_eskf_node src/uwb_eskf_node.cpp)
target_include_directories(uwb_eskf_node PRIVATE include)
target_link_libraries(uwb_eskf_node Eigen3::Eigen)
ament_target_dependencies(uwb_eskf_node rclcpp sensor_msgs nav_msgs std_msgs px4_msgs urdf ament_index_cpp)
install(TARGETS uwb_eskf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})  # uni.urdf + rig.txt

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test)  # ban deploy khong kem test/
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filter test/test_filter.cpp)
  target_include_directories(test_filter PRIVATE include)
  target_link_libraries(test_filter Eigen3::Eigen)
  target_compile_definitions(test_filter PRIVATE PARITY_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/parity.csv")
endif()

ament_package()
