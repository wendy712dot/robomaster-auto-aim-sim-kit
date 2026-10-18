#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>


struct EnemyConfig
{
    std::string color;
};


struct PreprocessConfig
{
    std::string method;
    int binary_threshold;
};


struct LightBarConfig
{
    double min_area;
    double min_ratio;
    double min_angle;
};


struct ArmorConfig
{
    double max_height_ratio;
    double max_y_ratio;

    double min_armor_aspect_ratio;
    double max_armor_aspect_ratio;
};

struct TargetConfig
{
    int tracking_threshold;
    int temporary_lost_threshold;
};

struct CameraConfig
{
    bool use_approximate_intrinsics;

    double fx_scale;
    double fy_scale;

    double k1;
    double k2;
    double p1;
    double p2;
    double k3;
};


struct ArmorSizeConfig
{
    double width_mm;
    double height_mm;
};


struct DebugConfig
{
    bool show_all_rects;
    bool show_filtered_light_bars;
    bool show_result;
    bool show_binary;
};


struct Config
{
    EnemyConfig enemy;

    PreprocessConfig preprocess;

    LightBarConfig light_bar;

    ArmorConfig armor;

    CameraConfig camera;

    ArmorSizeConfig armor_size;

    DebugConfig debug;

    TargetConfig target;
};


// 创建默认配置
Config createDefaultConfig();


// 从 YAML 文件读取配置
bool loadConfig(
    const std::string& file_path,
    Config& config
);


#endif