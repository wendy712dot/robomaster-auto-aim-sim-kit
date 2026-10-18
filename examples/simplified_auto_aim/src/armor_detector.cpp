#include "armor_detector.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>


// ============================================================
// 构造函数
// ============================================================

ArmorDetector::ArmorDetector(
    const EnemyConfig& enemy_config,
    const PreprocessConfig& preprocess_config,
    const LightBarConfig& light_bar_config,
    const ArmorConfig& armor_config
)
    : preprocess_config_(preprocess_config),
      light_bar_config_(light_bar_config),
      armor_config_(armor_config),
      enemy_config_(enemy_config)
{
}


// ============================================================
// 图像预处理
//
// 支持：
// 1. gray_threshold：灰度图 + 固定阈值二值化
// 2. color_difference：颜色通道差 + 固定阈值二值化
// ============================================================

cv::Mat ArmorDetector::preprocess(
    const cv::Mat& frame
)
{
    cv::Mat binary;


    // ========================================================
    // 方法 1：灰度图 + 固定阈值二值化
    // ========================================================

    if (preprocess_config_.method == "gray_threshold")
    {
        cv::Mat gray;

        cv::cvtColor(
            frame,
            gray,
            cv::COLOR_BGR2GRAY
        );

        cv::threshold(
            gray,
            binary,
            preprocess_config_.binary_threshold,
            255,
            cv::THRESH_BINARY
        );

        return binary;
    }


    // ========================================================
    // 方法 2：颜色通道差
    //
    // blue:
    //     B - R
    //
    // red:
    //     R - B
    // ========================================================

    if (preprocess_config_.method == "color_difference")
    {
        std::vector<cv::Mat> channels;

        cv::split(
            frame,
            channels
        );

        cv::Mat color_difference;


        // ----------------------------------------------------
        // 蓝色目标
        // ----------------------------------------------------

        if (enemy_config_.color == "blue")
        {
            cv::subtract(
                channels[0],
                channels[2],
                color_difference
            );
        }


        // ----------------------------------------------------
        // 红色目标
        // ----------------------------------------------------

        else if (enemy_config_.color == "red")
        {
            cv::subtract(
                channels[2],
                channels[0],
                color_difference
            );
        }


        // ----------------------------------------------------
        // 未知颜色
        // ----------------------------------------------------

        else
        {
            std::cerr
                << "Unknown enemy color: "
                << enemy_config_.color
                << std::endl;

            return cv::Mat::zeros(
                frame.size(),
                CV_8UC1
            );
        }


        // ----------------------------------------------------
        // 对颜色差分图进行二值化
        // ----------------------------------------------------

        cv::threshold(
            color_difference,
            binary,
            preprocess_config_.binary_threshold,
            255,
            cv::THRESH_BINARY
        );

        return binary;
    }


    // ========================================================
    // 未知预处理方法
    // 回退到 gray_threshold
    // ========================================================

    std::cerr
        << "Unknown preprocess method: "
        << preprocess_config_.method
        << ", fallback to gray_threshold."
        << std::endl;

    cv::Mat gray;

    cv::cvtColor(
        frame,
        gray,
        cv::COLOR_BGR2GRAY
    );

    cv::threshold(
        gray,
        binary,
        preprocess_config_.binary_threshold,
        255,
        cv::THRESH_BINARY
    );

    return binary;
}


// ============================================================
// 查找轮廓
// ============================================================

std::vector<std::vector<cv::Point>>
ArmorDetector::findContours(
    const cv::Mat& binary
)
{
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        binary,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    return contours;
}


// ============================================================
// 根据轮廓计算最小旋转外接矩形
// ============================================================

std::vector<cv::RotatedRect>
ArmorDetector::getRotatedRects(
    const std::vector<std::vector<cv::Point>>& contours
)
{
    std::vector<cv::RotatedRect> rects;

    for (const auto& contour : contours)
    {
        if (contour.empty())
        {
            continue;
        }

        cv::RotatedRect rect =
            cv::minAreaRect(contour);

        rects.push_back(rect);
    }

    return rects;
}


// ============================================================
// 筛选灯条
// ============================================================

std::vector<cv::RotatedRect>
ArmorDetector::filterLightBars(
    const std::vector<cv::RotatedRect>& rects
)
{
    std::vector<cv::RotatedRect> light_bars;

    for (const auto& rect : rects)
    {
        float width =
            rect.size.width;

        float height =
            rect.size.height;

        float area =
            width * height;


        // ----------------------------------------------------
        // 1. 面积筛选
        // ----------------------------------------------------

        if (area < light_bar_config_.min_area)
        {
            continue;
        }


        // ----------------------------------------------------
        // 2. 计算灯条长边和短边
        // ----------------------------------------------------

        float long_side =
            std::max(width, height);

        float short_side =
            std::min(width, height);

        if (short_side <= 0.0f)
        {
            continue;
        }


        // ----------------------------------------------------
        // 3. 长宽比筛选
        // ----------------------------------------------------

        float ratio =
            long_side / short_side;

        // 实际视频中灯条受运动、倾斜和发光区域膨胀影响，
        // 部分真实灯条的长宽比会下降到约 1.6。
        if (ratio < light_bar_config_.min_ratio)
        {
            continue;
        }


        // ----------------------------------------------------
        // 4. 计算灯条长轴方向
        //
        // 最终统一为：
        // 0°  = 水平
        // 90° = 竖直
        // ----------------------------------------------------

        float angle =
            rect.angle;

        if (width < height)
        {
            angle += 90.0f;
        }

        while (angle < 0.0f)
        {
            angle += 180.0f;
        }

        while (angle >= 180.0f)
        {
            angle -= 180.0f;
        }

        if (angle > 90.0f)
        {
            angle =
                180.0f - angle;
        }


        // ----------------------------------------------------
        // 5. 角度筛选
        // ----------------------------------------------------

        if (angle < light_bar_config_.min_angle)
        {
            continue;
        }


        // 满足全部条件，保存为灯条候选
        light_bars.push_back(rect);
    }

    return light_bars;
}


// ============================================================
// 获取单根灯条的上端点和下端点
// ============================================================

void ArmorDetector::getLightEndpoints(
    const cv::RotatedRect& light,
    cv::Point2f& top,
    cv::Point2f& bottom
)
{
    cv::Point2f points[4];

    light.points(points);


    // 当前方法适用于近似竖直灯条。
    // 当灯条发生较大倾斜时，可进一步根据长轴方向计算端点。

    // 按 y 坐标从小到大排序
    // y 越小，在图像中越靠上
    std::sort(
        points,
        points + 4,
        [](const cv::Point2f& a,
           const cv::Point2f& b)
        {
            return a.y < b.y;
        }
    );


    // 上方两个角点的中点
    top =
        (points[0] + points[1]) *
        0.5f;


    // 下方两个角点的中点
    bottom =
        (points[2] + points[3]) *
        0.5f;
}


// ============================================================
// 灯条两两配对，生成装甲板候选
// ============================================================

std::vector<Armor>
ArmorDetector::matchArmors(
    const std::vector<cv::RotatedRect>& light_bars
)
{
    std::vector<Armor> armors;

    if (light_bars.size() < 2)
    {
        return armors;
    }

    struct PairCandidate
    {
        size_t first;
        size_t second;
        float score;
    };

    std::vector<PairCandidate> candidates;

    // 将 RotatedRect 的长轴角度统一到 0~90 度。
    auto longAxisAngle = [](const cv::RotatedRect& light)
    {
        float angle = light.angle;

        if (light.size.width < light.size.height)
        {
            angle += 90.0f;
        }

        while (angle < 0.0f)
        {
            angle += 180.0f;
        }

        while (angle >= 180.0f)
        {
            angle -= 180.0f;
        }

        if (angle > 90.0f)
        {
            angle = 180.0f - angle;
        }

        return angle;
    };

    constexpr float max_angle_difference = 15.0f;

    auto makeArmor = [this](
        const cv::RotatedRect& light1,
        const cv::RotatedRect& light2)
    {
        Armor armor;

        if (light1.center.x < light2.center.x)
        {
            armor.left_light = light1;
            armor.right_light = light2;
        }
        else
        {
            armor.left_light = light2;
            armor.right_light = light1;
        }

        armor.center =
            (armor.left_light.center + armor.right_light.center) * 0.5f;

        getLightEndpoints(
            armor.left_light,
            armor.left_top,
            armor.left_bottom);
        getLightEndpoints(
            armor.right_light,
            armor.right_top,
            armor.right_bottom);

        return armor;
    };

    // 由原有允许范围推导一个优选间距，不改变原来的硬性上下限。
    const float preferred_aspect_ratio = std::sqrt(
        static_cast<float>(
            armor_config_.min_armor_aspect_ratio *
            armor_config_.max_armor_aspect_ratio));

    // 先生成满足硬性几何约束的候选，并为每一对计算匹配分数。
    for (size_t i = 0; i < light_bars.size(); ++i)
    {
        for (size_t j = i + 1; j < light_bars.size(); ++j)
        {
            const auto& light1 = light_bars[i];
            const auto& light2 = light_bars[j];

            const float height1 = std::max(
                light1.size.width,
                light1.size.height);
            const float height2 = std::max(
                light2.size.width,
                light2.size.height);

            const float min_height = std::min(height1, height2);
            const float max_height = std::max(height1, height2);

            if (min_height <= 0.0f)
            {
                continue;
            }

            const float height_ratio = max_height / min_height;

            if (height_ratio > armor_config_.max_height_ratio)
            {
                continue;
            }

            const float average_height = (height1 + height2) * 0.5f;
            const float delta_x = std::abs(
                light1.center.x - light2.center.x);
            const float delta_y = std::abs(
                light1.center.y - light2.center.y);
            const float y_ratio = delta_y / average_height;
            const float armor_aspect_ratio = delta_x / average_height;

            if (y_ratio > armor_config_.max_y_ratio)
            {
                continue;
            }

            if (
                armor_aspect_ratio <
                    armor_config_.min_armor_aspect_ratio ||
                armor_aspect_ratio >
                    armor_config_.max_armor_aspect_ratio)
            {
                continue;
            }

            const float angle_difference = std::abs(
                longAxisAngle(light1) - longAxisAngle(light2));

            if (angle_difference > max_angle_difference)
            {
                continue;
            }

            // 分数越低越合理：优先高度接近、水平对齐、方向平行，
            // 且灯条间距接近期望装甲比例的组合。
            const float score =
                2.0f * (height_ratio - 1.0f) +
                2.0f * y_ratio +
                angle_difference / max_angle_difference +
                std::abs(
                    armor_aspect_ratio - preferred_aspect_ratio) /
                    preferred_aspect_ratio;

            candidates.push_back({i, j, score});
        }
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const PairCandidate& lhs, const PairCandidate& rhs)
        {
            return lhs.score < rhs.score;
        });

    // 按得分从好到坏进行一对一匹配。
    // 一根灯条一旦被采用，就不能再与其他灯条组成交叉候选。
    std::vector<bool> light_used(light_bars.size(), false);

    for (const auto& candidate : candidates)
    {
        if (
            light_used[candidate.first] ||
            light_used[candidate.second])
        {
            continue;
        }

        const auto& light1 = light_bars[candidate.first];
        const auto& light2 = light_bars[candidate.second];

        Armor armor = makeArmor(light1, light2);

        armors.push_back(armor);
        light_used[candidate.first] = true;
        light_used[candidate.second] = true;
    }

    return armors;
}
