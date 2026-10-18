cmake_minimum_required(VERSION 3.10)

project(simplified_auto_aim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)


# 主程序
add_executable(
    auto_aim
    src/main.cpp
    src/auto_aim_core.cpp
    src/video_reader.cpp
    src/armor_detector.cpp
    src/pose_solver.cpp
    src/config.cpp
    src/target_selector.cpp
    src/debug_visualizer.cpp
    src/communication.cpp
    src/udp_sender_class.cpp
)

target_include_directories(
    auto_aim
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
    auto_aim
    PRIVATE
    ${OpenCV_LIBS}
)


# 通信协议测试
add_executable(
    communication_test
    src/communication_test.cpp
    src/communication.cpp
)

target_include_directories(
    communication_test
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
    communication_test
    PRIVATE
    ${OpenCV_LIBS}
)


# UDP 接收端
add_executable(
    udp_receiver
    src/udp_receiver.cpp
    src/communication.cpp
)

target_include_directories(
    udp_receiver
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
    udp_receiver
    PRIVATE
    ${OpenCV_LIBS}
)


# UDP 独立发送测试
add_executable(
    udp_sender
    src/udp_sender.cpp
    src/communication.cpp
)

target_include_directories(
    udp_sender
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
    udp_sender
    PRIVATE
    ${OpenCV_LIBS}
)
