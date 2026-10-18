#include "auto_aim_core.hpp"

AutoAimCore::AutoAimCore(const Config& config)
    : detector_(
          config.enemy,
          config.preprocess,
          config.light_bar,
          config.armor),
      selector_(config.target),
      pose_solver_(config.camera, config.armor_size)
{
}

AimResult AutoAimCore::process(const cv::Mat& frame)
{
    AimResult result;

    if (frame.empty())
    {
        return result;
    }

    result.binary = detector_.preprocess(frame);

    const auto contours = detector_.findContours(result.binary);
    result.rects = detector_.getRotatedRects(contours);
    result.light_bars = detector_.filterLightBars(result.rects);
    result.armors = detector_.matchArmors(result.light_bars);
    result.target = selector_.select(result.armors, frame.size());

    if (result.target.valid)
    {
        result.pose = pose_solver_.solve(
            result.target.armor,
            frame.size());
        result.target_valid = result.pose.success;
    }

    return result;
}
