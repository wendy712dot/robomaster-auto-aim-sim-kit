#include "pose_solver.hpp"

#include <cmath>


PoseSolver::PoseSolver(
    const CameraConfig& camera_config,
    const ArmorSizeConfig& armor_size_config
)
    : camera_config_(camera_config),
      armor_size_config_(armor_size_config)
{
    camera_matrix_ = cv::Mat();

    dist_coeffs_ = (
        cv::Mat_<double>(5, 1) <<
        camera_config_.k1,
        camera_config_.k2,
        camera_config_.p1,
        camera_config_.p2,
        camera_config_.k3
    );
}


void PoseSolver::updateCameraMatrix(
    const cv::Size& image_size
)
{
    double width =
        static_cast<double>(image_size.width);

    double height =
        static_cast<double>(image_size.height);

    double cx = width / 2.0;
    double cy = height / 2.0;

    // 当前阶段使用基于图像尺寸构造的近似相机内参
    double fx =
        width * camera_config_.fx_scale;

    double fy =
        width * camera_config_.fy_scale;

    camera_matrix_ = (
        cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    );
}


PoseResult PoseSolver::solve(
    const Armor& armor,
    const cv::Size& image_size
)
{
    PoseResult result;

    updateCameraMatrix(image_size);

    double half_width =
        armor_size_config_.width_mm / 2.0;

    double half_height =
        armor_size_config_.height_mm / 2.0;

    // 装甲板三维点：左上、右上、右下、左下
    std::vector<cv::Point3f> object_points =
    {
        {
            static_cast<float>(-half_width),
            static_cast<float>(-half_height),
            0.0f
        },
        {
            static_cast<float>(half_width),
            static_cast<float>(-half_height),
            0.0f
        },
        {
            static_cast<float>(half_width),
            static_cast<float>(half_height),
            0.0f
        },
        {
            static_cast<float>(-half_width),
            static_cast<float>(half_height),
            0.0f
        }
    };

    std::vector<cv::Point2f> image_points =
    {
        armor.left_top,
        armor.right_top,
        armor.right_bottom,
        armor.left_bottom
    };

    cv::Vec3d rvec;
    cv::Vec3d tvec;

    bool success =
        cv::solvePnP(
            object_points,
            image_points,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_ITERATIVE
        );

    if (!success)
    {
        return result;
    }

    result.success = true;
    result.rvec = rvec;
    result.tvec = tvec;

    double x = tvec[0];
    double y = tvec[1];
    double z = tvec[2];

    result.distance =
        std::sqrt(
            x * x +
            y * y +
            z * z
        );

    result.yaw =
        std::atan2(x, z) *
        180.0 / CV_PI;

    result.pitch =
        std::atan2(
            -y,
            std::sqrt(x * x + z * z)
        ) * 180.0 / CV_PI;

    return result;
}