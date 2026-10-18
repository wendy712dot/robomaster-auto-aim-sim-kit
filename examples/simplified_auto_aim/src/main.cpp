#include <iostream>
#include <iomanip>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "auto_aim_core.hpp"
#include "video_reader.hpp"
#include "config.hpp"
#include "debug_visualizer.hpp"
#include "communication.hpp"
#include "udp_sender.hpp"


int main()
{
    // 0. 加载配置文件
    Config config = createDefaultConfig();

    if (!loadConfig("../config/config.yaml", config))
    {
        std::cerr
            << "Failed to load config.yaml, "
            << "using default configuration."
            << std::endl;
    }
    else
    {
        std::cout
            << "Config loaded successfully!"
            << std::endl;
    }


    // 1. 打开测试视频
    VideoReader reader("../videos/test.mp4");

    if (!reader.isOpened())
    {
        std::cerr
            << "Failed to open video!"
            << std::endl;

        return -1;
    }


    // 2. 创建统一自瞄核心和输出模块
    AutoAimCore auto_aim(config);

    DebugVisualizer visualizer(
        config.debug,
        reader.getFPS()
    );

    Communication communication;

    UdpSender udp_sender(
        "127.0.0.1",
        9000
    );

    if (!udp_sender.isValid())
    {
        std::cerr
            << "Failed to create UDP sender."
            << std::endl;

        return -1;
    }

    std::cout
        << "Video opened successfully!"
        << std::endl;


    // 3. FPS 相关变量
    auto previous_time =
        std::chrono::steady_clock::now();

    double fps = 0.0;


    // 4. 主循环
    while (true)
    {
        cv::Mat frame =
            reader.read();

        if (frame.empty())
        {
            std::cout
                << "Video finished."
                << std::endl;

            break;
        }


        // 计算 FPS
        auto current_time =
            std::chrono::steady_clock::now();

        double elapsed =
            std::chrono::duration<double>(
                current_time - previous_time
            ).count();

        previous_time =
            current_time;

        if (elapsed > 0.0)
        {
            fps =
                1.0 / elapsed;
        }


        // 输入层统一提供 cv::Mat；核心算法统一返回 AimResult。
        AimResult aim_result = auto_aim.process(frame);


        // 通信编码与模拟接收
        double send_yaw = 0.0;
        double send_pitch = 0.0;
        double send_distance = 0.0;

        if (aim_result.target_valid)
        {
            send_yaw = aim_result.pose.yaw;
            send_pitch = aim_result.pose.pitch;
            send_distance = aim_result.pose.distance;
        }

        CanFrame tx_frame =
            communication.encode(
                send_yaw,
                send_pitch,
                send_distance,
                aim_result.target.status
            );

        if (!udp_sender.send(tx_frame))
        {
            std::cerr
                << "Failed to send UDP frame."
                << std::endl;
        }

        CommunicationData rx_data =
            communication.decode(
                tx_frame
            );


        std::cout
            << "[TX] ID: 0x"
            << std::hex
            << std::uppercase
            << tx_frame.id
            << " Data: ";

        for (uint8_t byte : tx_frame.data)
        {
            std::cout
                << std::setw(2)
                << std::setfill('0')
                << static_cast<int>(byte)
                << " ";
        }

        std::cout
            << std::dec
            << std::endl;


        std::cout
            << std::fixed
            << std::setprecision(2)
            << "[RX] "
            << "Yaw: "
            << rx_data.yaw
            << " deg, "
            << "Pitch: "
            << rx_data.pitch
            << " deg, "
            << "Distance: "
            << rx_data.distance
            << " mm, "
            << "Status: ";

        switch (rx_data.status)
        {
            case TargetStatus::NO_TARGET:
                std::cout << "NO_TARGET";
                break;

            case TargetStatus::DETECTED:
                std::cout << "DETECTED";
                break;

            case TargetStatus::TRACKING:
                std::cout << "TRACKING";
                break;

            case TargetStatus::TEMP_LOST:
                std::cout << "TEMP_LOST";
                break;
        }

        std::cout
            << ", Checksum: "
            << (
                rx_data.checksum_valid
                ? "OK"
                : "ERROR"
            )
            << std::endl;

        // 调试显示
        visualizer.show(
            frame,
            aim_result.binary,
            aim_result.rects,
            aim_result.light_bars,
            aim_result.armors,
            aim_result.target,
            aim_result.pose,
            aim_result.target_valid,
            fps
        );


        // ESC 退出
        int key =
            cv::waitKey(30);

        if (key == 27)
        {
            break;
        }
    }


    // 5. 释放资源
    visualizer.releaseVideoWriter();

    cv::destroyAllWindows();

    return 0;
}
