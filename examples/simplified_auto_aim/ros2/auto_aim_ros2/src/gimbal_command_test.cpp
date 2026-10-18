#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/vision_send_data.hpp>

using namespace std::chrono_literals;

class GimbalCommandTest : public rclcpp::Node
{
public:
    GimbalCommandTest()
        : Node("gimbal_command_test")
    {
        yaw_ = declare_parameter<double>("yaw", 0.0);
        pitch_ = declare_parameter<double>("pitch", 0.0);
        distance_ = declare_parameter<double>("distance", 1.0);
        target_state_ =
            declare_parameter<int>("target_state", 1);

        // 与仿真器订阅端的 BEST_EFFORT QoS 保持一致。
        publisher_ =
            create_publisher<rm_interfaces::msg::VisionSendData>(
                "/vision_send_data",
                rclcpp::SensorDataQoS().keep_last(1));

        timer_ = create_wall_timer(
            100ms,
            std::bind(
                &GimbalCommandTest::publishCommand,
                this));

        RCLCPP_INFO(
            get_logger(),
            "Publishing gimbal command: "
            "yaw=%.2f deg, pitch=%.2f deg, "
            "distance=%.2f, target_state=%d",
            yaw_,
            pitch_,
            distance_,
            target_state_);
    }

private:
    void publishCommand()
    {
        rm_interfaces::msg::VisionSendData message;

        message.header.stamp = now();
        message.header.frame_id = "gimbal_link";

        message.target_state =
            static_cast<std::uint8_t>(target_state_);
        message.target_type = 0;

        message.yaw = static_cast<float>(yaw_);
        message.pitch = static_cast<float>(pitch_);
        message.delta_yaw = 0.0F;
        message.delta_pitch = 0.0F;
        message.target_distance =
            static_cast<float>(distance_);

        message.vel_x = 0.0F;
        message.vel_y = 0.0F;
        message.vel_yaw = 0.0F;
        message.control_id = 0.0F;

        publisher_->publish(message);
    }

    double yaw_ = 0.0;
    double pitch_ = 0.0;
    double distance_ = 1.0;
    int target_state_ = 1;

    rclcpp::Publisher<
        rm_interfaces::msg::VisionSendData>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<GimbalCommandTest>());
    rclcpp::shutdown();
    return 0;
}