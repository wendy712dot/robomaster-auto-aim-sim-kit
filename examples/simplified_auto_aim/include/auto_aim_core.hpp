#ifndef AUTO_AIM_CORE_HPP
#define AUTO_AIM_CORE_HPP

#include <opencv2/opencv.hpp>

#include "armor_detector.hpp"
#include "config.hpp"
#include "pose_solver.hpp"
#include "target_selector.hpp"

struct AimResult
{
    cv::Mat binary;
    std::vector<cv::RotatedRect> rects;
    std::vector<cv::RotatedRect> light_bars;
    std::vector<Armor> armors;
    TargetResult target;
    PoseResult pose;
    bool target_valid = false;
};

class AutoAimCore
{
public:
    explicit AutoAimCore(const Config& config);

    AimResult process(const cv::Mat& frame);

private:
    ArmorDetector detector_;
    TargetSelector selector_;
    PoseSolver pose_solver_;
};

#endif
