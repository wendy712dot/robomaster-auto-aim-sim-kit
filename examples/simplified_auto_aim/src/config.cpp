#include "config.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

Config createDefaultConfig()
{
    Config config;


    // ==========================================
    // 敌方颜色
    // ==========================================

    config.enemy.color = "blue";


    // ==========================================
    // 图像预处理
    // ==========================================

    config.preprocess.method = "gray_threshold";
    config.preprocess.binary_threshold = 150;


    // ==========================================
    // 灯条筛选参数
    // ==========================================

    config.light_bar.min_area = 10.0;
    config.light_bar.min_ratio = 2.0;
    config.light_bar.min_angle = 60.0;


    // ==========================================
    // Armor 配对参数
    // ==========================================

    config.armor.max_height_ratio = 1.5;
    config.armor.max_y_ratio = 0.5;

    config.armor.min_armor_aspect_ratio = 1.5;
    config.armor.max_armor_aspect_ratio = 5.0;

    // ==========================================
    // 目标状态参数
    // ==========================================

    config.target.tracking_threshold = 3;
    config.target.temporary_lost_threshold = 5;

    // ==========================================
    // 相机参数
    // ==========================================

    config.camera.use_approximate_intrinsics = true;

    config.camera.fx_scale = 1.0;
    config.camera.fy_scale = 1.0;

    config.camera.k1 = 0.0;
    config.camera.k2 = 0.0;
    config.camera.p1 = 0.0;
    config.camera.p2 = 0.0;
    config.camera.k3 = 0.0;


    // ==========================================
    // 装甲板真实尺寸
    // 单位：mm
    // ==========================================

    config.armor_size.width_mm = 135.0;
    config.armor_size.height_mm = 55.0;


    // ==========================================
    // 调试显示
    // ==========================================

    config.debug.show_binary = true;
    config.debug.show_all_rects = true;
    config.debug.show_filtered_light_bars = true;
    config.debug.show_result = true;


    return config;
}

bool loadConfig(
    const std::string& file_path,
    Config& config
)
{
    cv::FileStorage fs(
        file_path,
        cv::FileStorage::READ
    );

    if (!fs.isOpened())
    {
        std::cerr
            << "Failed to open config file: "
            << file_path
            << std::endl;

        return false;
    }


    // ==========================================
    // 敌方颜色
    // ==========================================

    fs["enemy"]["color"]
        >> config.enemy.color;


    // ==========================================
    // 图像预处理
    // ==========================================

    fs["preprocess"]["method"]
        >> config.preprocess.method;

    fs["preprocess"]["binary_threshold"]
        >> config.preprocess.binary_threshold;


    // ==========================================
    // 灯条筛选参数
    // ==========================================

    fs["light_bar"]["min_area"]
        >> config.light_bar.min_area;

    fs["light_bar"]["min_ratio"]
        >> config.light_bar.min_ratio;

    fs["light_bar"]["min_angle"]
        >> config.light_bar.min_angle;


    // ==========================================
    // Armor 配对参数
    // ==========================================

    fs["armor"]["max_height_ratio"]
        >> config.armor.max_height_ratio;

    fs["armor"]["max_y_ratio"]
        >> config.armor.max_y_ratio;

    fs["armor"]["min_armor_aspect_ratio"] >>
        config.armor.min_armor_aspect_ratio;

    fs["armor"]["max_armor_aspect_ratio"] >>
        config.armor.max_armor_aspect_ratio;

    fs["target"]["tracking_threshold"] >>
        config.target.tracking_threshold;

    fs["target"]["temporary_lost_threshold"] >>
        config.target.temporary_lost_threshold;


    // ==========================================
    // 相机参数
    // ==========================================

    int use_approximate_intrinsics = 1;

    fs["camera"]["use_approximate_intrinsics"]
        >> use_approximate_intrinsics;

    config.camera.use_approximate_intrinsics =
        (use_approximate_intrinsics != 0);

    fs["camera"]["fx_scale"]
        >> config.camera.fx_scale;

    fs["camera"]["fy_scale"]
        >> config.camera.fy_scale;

    fs["camera"]["k1"]
        >> config.camera.k1;

    fs["camera"]["k2"]
        >> config.camera.k2;

    fs["camera"]["p1"]
        >> config.camera.p1;

    fs["camera"]["p2"]
        >> config.camera.p2;

    fs["camera"]["k3"]
        >> config.camera.k3;


    // ==========================================
    // 装甲板尺寸
    // ==========================================

    fs["armor_size"]["width_mm"]
        >> config.armor_size.width_mm;

    fs["armor_size"]["height_mm"]
        >> config.armor_size.height_mm;


    // ==========================================
    // 调试显示
    // ==========================================

    int show_binary = 1;
    int show_all_rects = 1;
    int show_filtered_light_bars = 1;
    int show_result = 1;

    fs["debug"]["show_binary"]
        >> show_binary;

    fs["debug"]["show_all_rects"]
        >> show_all_rects;

    fs["debug"]["show_filtered_light_bars"]
        >> show_filtered_light_bars;

    fs["debug"]["show_result"]
        >> show_result;

    config.debug.show_binary =
        (show_binary != 0);

    config.debug.show_all_rects =
        (show_all_rects != 0);

    config.debug.show_filtered_light_bars =
        (show_filtered_light_bars != 0);

    config.debug.show_result =
        (show_result != 0);


    fs.release();

    return true;
}