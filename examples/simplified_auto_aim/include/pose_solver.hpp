#ifndef POSE_SOLVER_HPP
#define POSE_SOLVER_HPP

#include <opencv2/opencv.hpp>

#include "armor_detector.hpp"
#include "config.hpp"

struct PoseResult
{
    bool success = false;

    cv::Vec3d rvec;
    cv::Vec3d tvec;

    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;
};

class PoseSolver
{
public:
    PoseSolver(
        const CameraConfig& camera_config,
        const ArmorSizeConfig& armor_size_config
    );

    PoseResult solve(
        const Armor& armor,
        const cv::Size& image_size
    );

private:
    void updateCameraMatrix(
        const cv::Size& image_size
    );

    CameraConfig camera_config_;
    ArmorSizeConfig armor_size_config_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

#endif