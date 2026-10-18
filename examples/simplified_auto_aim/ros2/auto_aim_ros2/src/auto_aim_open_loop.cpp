#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/vision_receive_data.hpp>
#include <rm_interfaces/msg/vision_send_data.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "auto_aim_core.hpp"
#include "config.hpp"

class AutoAimOpenLoop : public rclcpp::Node
{
public:
    AutoAimOpenLoop()
        : Node("auto_aim_open_loop"),
          previous_time_(std::chrono::steady_clock::now())
    {
        Config config = createDefaultConfig();

        const std::string config_path =
            ament_index_cpp::get_package_share_directory(
                "auto_aim_ros2") +
            "/config/config.yaml";

        if (!loadConfig(config_path, config))
        {
            RCLCPP_WARN(
                get_logger(),
                "Failed to load config; using defaults.");
        }


        // 仿真相机内参：
        // fx = fy = 1303.675283386667，图像宽度为 1440。
        config.camera.fx_scale =
            1303.675283386667 / 1440.0;
        config.camera.fy_scale =
            1303.675283386667 / 1440.0;

        auto_aim_ = std::make_unique<AutoAimCore>(config);

        control_enabled_ =
            declare_parameter<bool>("enable_control", false);
        max_correction_deg_ =
            declare_parameter<double>("max_correction_deg", 2.0);
        control_deadband_deg_ =
            declare_parameter<double>("control_deadband_deg", 0.15);
        command_period_ms_ =
            declare_parameter<int>("command_period_ms", 50);
        image_timeout_ms_ =
            declare_parameter<int>("image_timeout_ms", 200);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        qos.durability_volatile();

        image_subscription_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/image_raw",
                qos,
                std::bind(
                    &AutoAimOpenLoop::imageCallback,
                    this,
                    std::placeholders::_1));

        // 仿真器以 BEST_EFFORT 发布和接收视觉控制消息。
        vision_feedback_subscription_ =
            create_subscription<rm_interfaces::msg::VisionReceiveData>(
                "/vision_receive_data",
                rclcpp::SensorDataQoS().keep_last(1),
                std::bind(
                    &AutoAimOpenLoop::visionFeedbackCallback,
                    this,
                    std::placeholders::_1));

        vision_command_publisher_ =
            create_publisher<rm_interfaces::msg::VisionSendData>(
                "/vision_send_data",
                rclcpp::SensorDataQoS().keep_last(1));

        image_watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(
                &AutoAimOpenLoop::imageWatchdogCallback,
                this));

        cv::namedWindow(
            "ROS2 Auto Aim Open Loop",
            cv::WINDOW_NORMAL);

        cv::resizeWindow(
            "ROS2 Auto Aim Open Loop",
            960,
            720);

        cv::namedWindow(
            "ROS2 Auto Aim Binary",
            cv::WINDOW_NORMAL);

        cv::resizeWindow(
            "ROS2 Auto Aim Binary",
            960,
            720);

        RCLCPP_INFO(
            get_logger(),
            "Waiting for simulator images on /image_raw; control=%s",
            control_enabled_ ? "ON" : "OFF");
    }

    ~AutoAimOpenLoop() override
    {
        cv::destroyAllWindows();
    }

private:
    void publishInvalidCommand()
    {
        if (!control_enabled_ || !has_gimbal_feedback_)
        {
            return;
        }

        rm_interfaces::msg::VisionSendData command;
        command.header.stamp = this->now();
        command.header.frame_id = "gimbal_link";
        command.target_state = 0;
        command.target_type = 0;
        command.pitch = static_cast<float>(current_pitch_deg_);
        command.yaw = static_cast<float>(current_yaw_deg_);
        command.delta_pitch = 0.0F;
        command.delta_yaw = 0.0F;
        command.target_distance = -1.0F;
        command.vel_x = 0.0F;
        command.vel_y = 0.0F;
        command.vel_yaw = 0.0F;
        command.control_id = static_cast<float>(++control_id_);
        vision_command_publisher_->publish(command);
    }

    void imageWatchdogCallback()
    {
        if (!control_enabled_ || !received_image_)
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_image_time_)
                .count();

        if (elapsed_ms <= image_timeout_ms_)
        {
            image_timeout_active_ = false;
            return;
        }

        if (!image_timeout_active_)
        {
            image_timeout_active_ = true;
            RCLCPP_WARN(
                get_logger(),
                "Image timeout after %d ms; holding current gimbal pose.",
                image_timeout_ms_);
        }

        publishInvalidCommand();
    }

    void visionFeedbackCallback(
        const rm_interfaces::msg::VisionReceiveData::SharedPtr msg)
    {
        current_yaw_deg_ = msg->yaw;
        current_pitch_deg_ = msg->pitch;
        has_gimbal_feedback_ = true;
    }

    double limitedCorrection(double error_deg) const
    {
        if (std::abs(error_deg) < control_deadband_deg_)
        {
            return 0.0;
        }

        return std::max(
            -max_correction_deg_,
            std::min(max_correction_deg_, error_deg));
    }

    void publishControlCommand(
        const TargetResult& target,
        const PoseResult& pose,
        const std::chrono::steady_clock::time_point& now)
    {
        if (!control_enabled_)
        {
            return;
        }

        if (!has_gimbal_feedback_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Control enabled, but no /vision_receive_data received.");
            return;
        }

        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_command_time_)
                .count();

        if (elapsed_ms < command_period_ms_)
        {
            return;
        }

        last_command_time_ = now;

        rm_interfaces::msg::VisionSendData command;
        command.header.stamp = this->now();
        command.header.frame_id = "gimbal_link";
        command.target_type = 0;
        command.delta_pitch = 0.0F;
        command.delta_yaw = 0.0F;
        command.vel_x = 0.0F;
        command.vel_y = 0.0F;
        command.vel_yaw = 0.0F;
        command.control_id =
            static_cast<float>(++control_id_);

        const bool valid_target =
            target.valid &&
            target.status == TargetStatus::TRACKING &&
            pose.success;

        if (valid_target)
        {
            // PoseSolver 输出的是相机画面内的相对误差；仿真器接收
            // 世界坐标系绝对角度，且其正方向与图像误差方向相反。
            const double yaw_correction =
                limitedCorrection(pose.yaw);
            const double pitch_correction =
                limitedCorrection(pose.pitch);

            command.target_state = 1;
            command.yaw = static_cast<float>(
                current_yaw_deg_ - yaw_correction);
            command.pitch = static_cast<float>(
                current_pitch_deg_ - pitch_correction);
            command.target_distance = static_cast<float>(
                pose.distance / 1000.0);
        }
        else
        {
            // 无可靠目标时发送无效状态，并保持当前姿态。
            command.target_state = 0;
            command.yaw = static_cast<float>(current_yaw_deg_);
            command.pitch = static_cast<float>(current_pitch_deg_);
            command.target_distance = -1.0F;
        }

        vision_command_publisher_->publish(command);
    }

    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv::Mat frame;

        try
        {
            frame = cv_bridge::toCvCopy(
                        msg,
                        sensor_msgs::image_encodings::BGR8)
                        ->image;
        }
        catch (const cv_bridge::Exception &error)
        {
            RCLCPP_ERROR(
                get_logger(),
                "cv_bridge conversion failed: %s",
                error.what());
            return;
        }

        if (frame.empty())
        {
            return;
        }

        const auto now =
            std::chrono::steady_clock::now();

        received_image_ = true;
        image_timeout_active_ = false;
        last_image_time_ = now;

        const double elapsed =
            std::chrono::duration<double>(
                now - previous_time_)
                .count();

        previous_time_ = now;

        const double fps =
            elapsed > 0.0 ? 1.0 / elapsed : 0.0;

        AimResult aim_result = auto_aim_->process(frame);

        publishControlCommand(
            aim_result.target,
            aim_result.pose,
            now);

        cv::Mat result = frame.clone();

        // 用绿色矩形显示通过筛选的灯条。
        for (const auto &light : aim_result.light_bars)
        {
            cv::Point2f points[4];
            light.points(points);

            for (int i = 0; i < 4; ++i)
            {
                cv::line(
                    result,
                    points[i],
                    points[(i + 1) % 4],
                    cv::Scalar(0, 255, 0),
                    2);
            }
        }

        for (const auto &armor : aim_result.armors)
        {
            cv::line(
                result,
                armor.left_top,
                armor.right_top,
                cv::Scalar(0, 255, 255),
                2);
            cv::line(
                result,
                armor.right_top,
                armor.right_bottom,
                cv::Scalar(0, 255, 255),
                2);
            cv::line(
                result,
                armor.right_bottom,
                armor.left_bottom,
                cv::Scalar(0, 255, 255),
                2);
            cv::line(
                result,
                armor.left_bottom,
                armor.left_top,
                cv::Scalar(0, 255, 255),
                2);
        }

        if (aim_result.target.valid)
        {
            cv::drawMarker(
                result,
                aim_result.target.armor.center,
                cv::Scalar(0, 0, 255),
                cv::MARKER_CROSS,
                30,
                2);
        }

        std::ostringstream status_text;
        status_text
            << std::fixed
            << std::setprecision(1)
            << "FPS: " << fps
            << "  Rects: " << aim_result.rects.size()
            << "  Lights: " << aim_result.light_bars.size()
            << "  Armors: " << aim_result.armors.size()
            << "  Control: "
            << (control_enabled_ ? "ON" : "OFF");

        cv::putText(
            result,
            status_text.str(),
            cv::Point(20, 35),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2);

        if (aim_result.pose.success)
        {
            std::ostringstream pose_text;
            pose_text
                << std::fixed
                << std::setprecision(2)
                << "Yaw: " << aim_result.pose.yaw
                << "  Pitch: " << aim_result.pose.pitch
                << "  Distance: " << aim_result.pose.distance << " mm";

            cv::putText(
                result,
                pose_text.str(),
                cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 0),
                2);
        }

        cv::imshow(
            "ROS2 Auto Aim Open Loop",
            result);

        cv::imshow(
            "ROS2 Auto Aim Binary",
            aim_result.binary);

        const int key = cv::waitKey(1);

        if (key == 27 || key == 'q' || key == 'Q')
        {
            rclcpp::shutdown();
        }
    }

    std::unique_ptr<AutoAimCore> auto_aim_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        image_subscription_;
    rclcpp::Subscription<
        rm_interfaces::msg::VisionReceiveData>::SharedPtr
        vision_feedback_subscription_;
    rclcpp::Publisher<
        rm_interfaces::msg::VisionSendData>::SharedPtr
        vision_command_publisher_;

    bool control_enabled_ = false;
    bool has_gimbal_feedback_ = false;
    double max_correction_deg_ = 2.0;
    double control_deadband_deg_ = 0.15;
    int command_period_ms_ = 50;
    int image_timeout_ms_ = 200;
    double current_yaw_deg_ = 0.0;
    double current_pitch_deg_ = 0.0;
    std::uint32_t control_id_ = 0;
    bool received_image_ = false;
    bool image_timeout_active_ = false;

    std::chrono::steady_clock::time_point previous_time_;
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point last_image_time_{};
    rclcpp::TimerBase::SharedPtr image_watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<AutoAimOpenLoop>());
    rclcpp::shutdown();
    return 0;
}
