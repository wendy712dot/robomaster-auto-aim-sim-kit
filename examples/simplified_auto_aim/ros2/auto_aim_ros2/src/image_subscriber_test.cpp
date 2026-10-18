#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriberTest : public rclcpp::Node
{
public:
    ImageSubscriberTest()
        : Node("image_subscriber_test"),
          last_fps_time_(std::chrono::steady_clock::now())
    {
        // 仿真器使用 RELIABLE，队列只保留最新一帧，避免积压旧画面。
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        qos.durability_volatile();

        subscription_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/image_raw",
                qos,
                std::bind(
                    &ImageSubscriberTest::imageCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Waiting for images on /image_raw...");
    }

    ~ImageSubscriberTest() override
    {
        cv::destroyAllWindows();
    }

private:
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        ++frame_count_;

        if (!received_first_frame_)
        {
            received_first_frame_ = true;

            RCLCPP_INFO(
                get_logger(),
                "First frame: width=%u height=%u encoding=%s "
                "stamp=%d.%09u",
                msg->width,
                msg->height,
                msg->encoding.c_str(),
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);
        }

        try
        {
            // 仿真器发布 rgb8；这里统一转换为 OpenCV 常用的 BGR8。
            const auto cv_ptr = cv_bridge::toCvCopy(
                msg,
                sensor_msgs::image_encodings::BGR8);

            if (cv_ptr->image.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Received an empty image.");
                return;
            }

            cv::imshow(
                "ROS2 Image Subscriber Test",
                cv_ptr->image);

            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
            {
                rclcpp::shutdown();
                return;
            }
        }
        catch (const cv_bridge::Exception &error)
        {
            RCLCPP_ERROR(
                get_logger(),
                "cv_bridge conversion failed: %s",
                error.what());
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(
                now - last_fps_time_)
                .count();

        if (elapsed >= 1.0)
        {
            const double fps =
                static_cast<double>(frame_count_) / elapsed;

            RCLCPP_INFO(
                get_logger(),
                "Receive FPS: %.2f",
                fps);

            frame_count_ = 0;
            last_fps_time_ = now;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        subscription_;

    bool received_first_frame_ = false;
    std::size_t frame_count_ = 0;
    std::chrono::steady_clock::time_point last_fps_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<ImageSubscriberTest>());
    rclcpp::shutdown();
    return 0;
}