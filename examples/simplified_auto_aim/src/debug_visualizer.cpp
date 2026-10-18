#include "debug_visualizer.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>


DebugVisualizer::DebugVisualizer(
    const DebugConfig& debug_config,
    double output_fps
)
    : debug_config_(debug_config),
      video_writer_initialized_(false),
      output_fps_(output_fps)
{
    if (output_fps_ <= 0.0)
    {
        output_fps_ = 30.0;
    }
}


void DebugVisualizer::show(
    const cv::Mat& frame,
    const cv::Mat& binary,
    const std::vector<cv::RotatedRect>& all_rects,
    const std::vector<cv::RotatedRect>& light_bars,
    const std::vector<Armor>& armors,
    const TargetResult& target,
    const PoseResult& pose,
    bool pose_valid,
    double fps
)
{
    // 二值化结果
    if (debug_config_.show_binary)
    {
        cv::imshow(
            "Binary",
            binary
        );
    }


    // 所有旋转矩形
    if (debug_config_.show_all_rects)
    {
        cv::Mat all_rects_display =
            frame.clone();

        for (const auto& rect : all_rects)
        {
            cv::Point2f points[4];

            rect.points(points);

            for (int i = 0; i < 4; ++i)
            {
                cv::line(
                    all_rects_display,
                    points[i],
                    points[(i + 1) % 4],
                    cv::Scalar(255, 0, 255),
                    2
                );
            }
        }

        cv::imshow(
            "All Rotated Rects",
            all_rects_display
        );
    }


    // 筛选后的灯条
    if (debug_config_.show_filtered_light_bars)
    {
        cv::Mat light_bars_display =
            frame.clone();

        for (const auto& rect : light_bars)
        {
            cv::Point2f points[4];

            rect.points(points);

            for (int i = 0; i < 4; ++i)
            {
                cv::line(
                    light_bars_display,
                    points[i],
                    points[(i + 1) % 4],
                    cv::Scalar(0, 255, 255),
                    2
                );
            }
        }

        cv::imshow(
            "Filtered Light Bars",
            light_bars_display
        );
    }


    // 最终结果
    if (debug_config_.show_result)
    {
        cv::Mat result_display =
            frame.clone();


        // 所有装甲板候选
        for (const auto& armor : armors)
        {
            cv::line(
                result_display,
                armor.left_top,
                armor.right_top,
                cv::Scalar(0, 255, 255),
                2
            );

            cv::line(
                result_display,
                armor.right_top,
                armor.right_bottom,
                cv::Scalar(0, 255, 255),
                2
            );

            cv::line(
                result_display,
                armor.right_bottom,
                armor.left_bottom,
                cv::Scalar(0, 255, 255),
                2
            );

            cv::line(
                result_display,
                armor.left_bottom,
                armor.left_top,
                cv::Scalar(0, 255, 255),
                2
            );
        }


        // 当前选中的最终目标
        if (target.valid)
        {
            const Armor& final_target =
                target.armor;

            cv::line(
                result_display,
                final_target.left_top,
                final_target.right_top,
                cv::Scalar(0, 255, 0),
                3
            );

            cv::line(
                result_display,
                final_target.right_top,
                final_target.right_bottom,
                cv::Scalar(0, 255, 0),
                3
            );

            cv::line(
                result_display,
                final_target.right_bottom,
                final_target.left_bottom,
                cv::Scalar(0, 255, 0),
                3
            );

            cv::line(
                result_display,
                final_target.left_bottom,
                final_target.left_top,
                cv::Scalar(0, 255, 0),
                3
            );

            // cv::circle(
            //     result_display,
            //     final_target.center,
            //     8,
            //     cv::Scalar(0, 0, 255),
            //     -1
            // );

            cv::drawMarker(
                result_display,
                final_target.center,
                cv::Scalar(0, 0, 255),
                cv::MARKER_CROSS,
                30,
                2
            );
        }


        // 位姿信息
        if (pose_valid)
        {
            std::ostringstream pose_text;

            pose_text
                << std::fixed
                << std::setprecision(2)
                << "Yaw: "
                << pose.yaw
                << " deg  "
                << "Pitch: "
                << pose.pitch
                << " deg  "
                << "Distance: "
                << pose.distance
                << " mm";

            cv::putText(
                result_display,
                pose_text.str(),
                cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2
            );
        }


        // FPS
        std::ostringstream fps_text;

        fps_text
            << std::fixed
            << std::setprecision(1)
            << "FPS: "
            << fps;

        cv::putText(
            result_display,
            fps_text.str(),
            cv::Point(20, 65),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );


        // 目标状态
        std::string status_text;

        switch (target.status)
        {
            case TargetStatus::NO_TARGET:
                status_text = "Status: NO_TARGET";
                break;

            case TargetStatus::DETECTED:
                status_text = "Status: DETECTED";
                break;

            case TargetStatus::TRACKING:
                status_text = "Status: TRACKING";
                break;

            case TargetStatus::TEMP_LOST:
                status_text = "Status: TEMP_LOST";
                break;
        }

        cv::putText(
            result_display,
            status_text,
            cv::Point(20, 95),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );


        cv::imshow(
            "Auto Aim Result",
            result_display
        );


        // 保存最终结果视频
        if (!video_writer_initialized_)
        {
            video_writer_.open(
                "../output/result.avi",
                cv::VideoWriter::fourcc(
                    'M',
                    'J',
                    'P',
                    'G'
                ),
                output_fps_,
                result_display.size()
            );

            if (video_writer_.isOpened())
            {
                video_writer_initialized_ = true;

                std::cout
                    << "Output video started: "
                    << "../output/result.avi"
                    << std::endl;
            }
            else
            {
                std::cerr
                    << "Failed to open output video."
                    << std::endl;
            }
        }


        if (video_writer_initialized_)
        {
            video_writer_.write(
                result_display
            );
        }
    }
}


void DebugVisualizer::releaseVideoWriter()
{
    if (video_writer_initialized_)
    {
        video_writer_.release();

        video_writer_initialized_ = false;
    }
}