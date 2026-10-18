#ifndef DEBUG_VISUALIZER_HPP
#define DEBUG_VISUALIZER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "armor_detector.hpp"
#include "target_selector.hpp"
#include "pose_solver.hpp"
#include "config.hpp"


class DebugVisualizer
{
public:
    DebugVisualizer(
        const DebugConfig& debug_config,
        double output_fps
    );

    void show(
        const cv::Mat& frame,
        const cv::Mat& binary,
        const std::vector<cv::RotatedRect>& rects,
        const std::vector<cv::RotatedRect>& light_bars,
        const std::vector<Armor>& armors,
        const TargetResult& target,
        const PoseResult& pose,
        bool pose_valid,
        double fps
    );

    void releaseVideoWriter();

private:
    DebugConfig debug_config_;

    double output_fps_;

    std::string targetStatusToString(
        TargetStatus status
    );

    cv::Scalar targetStatusColor(
        TargetStatus status
    );

    cv::VideoWriter video_writer_;

    bool video_writer_initialized_;
};


#endif