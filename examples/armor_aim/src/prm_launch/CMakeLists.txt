cmake_minimum_required(VERSION 3.5)
project(prm_launch)

# find dependencies
find_package(ament_cmake REQUIRED)

# 本仓库只支持并验证仿真实时链路。
install(
  PROGRAMS launch/sim_detector.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
