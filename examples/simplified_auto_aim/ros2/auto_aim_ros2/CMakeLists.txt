cmake_minimum_required(VERSION 3.8)
project(auto_aim_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# ROS2 工作区使用符号链接，因此先获取包的真实目录，
# 再由真实目录定位原自瞄项目根目录。
get_filename_component(
  AUTO_AIM_PACKAGE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}"
  REALPATH
)

get_filename_component(
  AUTO_AIM_ROOT
  "${AUTO_AIM_PACKAGE_DIR}/../.."
  ABSOLUTE
)

# 图像订阅测试
add_executable(
  image_subscriber_test
  src/image_subscriber_test.cpp
)

ament_target_dependencies(
  image_subscriber_test
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(
  image_subscriber_test
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  image_subscriber_test
  ${OpenCV_LIBS}
)

# 云台命令测试
add_executable(
  gimbal_command_test
  src/gimbal_command_test.cpp
)

ament_target_dependencies(
  gimbal_command_test
  rclcpp
  rm_interfaces
)

# ROS2 开环自瞄节点
add_executable(
  auto_aim_open_loop
  src/auto_aim_open_loop.cpp
  ${AUTO_AIM_ROOT}/src/auto_aim_core.cpp
  ${AUTO_AIM_ROOT}/src/armor_detector.cpp
  ${AUTO_AIM_ROOT}/src/target_selector.cpp
  ${AUTO_AIM_ROOT}/src/pose_solver.cpp
  ${AUTO_AIM_ROOT}/src/config.cpp
)

ament_target_dependencies(
  auto_aim_open_loop
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
  rm_interfaces
)

target_include_directories(
  auto_aim_open_loop
  PRIVATE
  ${AUTO_AIM_ROOT}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  auto_aim_open_loop
  ${OpenCV_LIBS}
)

install(
  TARGETS
  image_subscriber_test
  gimbal_command_test
  auto_aim_open_loop
  DESTINATION lib/${PROJECT_NAME}
)

# 将原项目配置文件安装到 ROS2 包的共享目录
install(
  FILES
  ${AUTO_AIM_ROOT}/config/config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
