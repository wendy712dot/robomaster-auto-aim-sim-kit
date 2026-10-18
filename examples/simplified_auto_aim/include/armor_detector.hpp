#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "config.hpp"


struct Armor
{
    cv::RotatedRect left_light;
    cv::RotatedRect right_light;

    cv::Point2f center;

    cv::Point2f left_top;
    cv::Point2f left_bottom;
    cv::Point2f right_top;
    cv::Point2f right_bottom;
};


class ArmorDetector
{
public:

    ArmorDetector(
        const EnemyConfig& enemy_config,
        const PreprocessConfig& preprocess_config,
        const LightBarConfig& light_bar_config,
        const ArmorConfig& armor_config
    );

    cv::Mat preprocess(
        const cv::Mat& frame
    );

    std::vector<std::vector<cv::Point>> findContours(
        const cv::Mat& binary
    );

    std::vector<cv::RotatedRect> getRotatedRects(
        const std::vector<std::vector<cv::Point>>& contours
    );

    std::vector<cv::RotatedRect> filterLightBars(
        const std::vector<cv::RotatedRect>& rects
    );

    std::vector<Armor> matchArmors(
        const std::vector<cv::RotatedRect>& light_bars
    );

    void getLightEndpoints(
        const cv::RotatedRect& light,
        cv::Point2f& top,
        cv::Point2f& bottom
    );


private:
    
    PreprocessConfig preprocess_config_;
    LightBarConfig light_bar_config_;
    ArmorConfig armor_config_;
    EnemyConfig enemy_config_;
    
    int binary_threshold_;
};


#endif
