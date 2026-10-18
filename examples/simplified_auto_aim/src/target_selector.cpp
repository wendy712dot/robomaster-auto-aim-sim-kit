#include "target_selector.hpp"


// ============================================================
// 构造函数
// ============================================================

TargetSelector::TargetSelector(
    const TargetConfig& target_config
)
    : detected_frames_(0),
      lost_frames_(0),
      had_target_(false),
      tracking_threshold_(
          target_config.tracking_threshold
      ),
      temporary_lost_threshold_(
          target_config.temporary_lost_threshold
      )
{
}


// ============================================================
// 目标选择与状态更新
//
// 目标选择策略：
// 1. 首次发现目标时，选择距离画面中心最近的候选；
// 2. 已存在历史目标时，选择距离上一帧目标最近的候选；
// 3. 短暂丢失时保留历史目标信息，不立即进入 NO_TARGET；
// 4. 连续丢失超过阈值后，清除目标状态。
// ============================================================

TargetResult TargetSelector::select(
    const std::vector<Armor>& armors,
    const cv::Size& image_size
)
{
    TargetResult result;


    // ==========================================
    // 情况 1：当前帧检测到装甲板
    // ==========================================

    if (!armors.empty())
    {
        cv::Point2f reference_center;


        // --------------------------------------
        // 1.1 确定目标选择的参考中心
        // --------------------------------------

        if (had_target_)
        {
            // 已有历史目标：
            // 优先选择距离上一帧目标中心最近的候选
            reference_center =
                last_target_center_;
        }
        else
        {
            // 首次发现目标：
            // 选择距离画面中心最近的候选
            reference_center =
                cv::Point2f(
                    image_size.width / 2.0f,
                    image_size.height / 2.0f
                );
        }


        // --------------------------------------
        // 1.2 寻找距离参考中心最近的候选
        // --------------------------------------

        int best_index = 0;

        double min_distance =
            cv::norm(
                armors[0].center -
                reference_center
            );

        for (
            int i = 1;
            i < static_cast<int>(armors.size());
            ++i
        )
        {
            double distance =
                cv::norm(
                    armors[i].center -
                    reference_center
                );

            if (distance < min_distance)
            {
                min_distance = distance;
                best_index = i;
            }
        }


        // --------------------------------------
        // 1.3 保存当前目标
        // --------------------------------------

        result.valid = true;

        result.armor =
            armors[best_index];

        // 保存目标中心，供下一帧目标选择使用
        last_target_center_ =
            result.armor.center;


        // --------------------------------------
        // 1.4 更新目标状态
        // --------------------------------------

        detected_frames_++;

        lost_frames_ = 0;
        had_target_ = true;

        if (
            detected_frames_ >=
            tracking_threshold_
        )
        {
            result.status =
                TargetStatus::TRACKING;
        }
        else
        {
            result.status =
                TargetStatus::DETECTED;
        }


        return result;
    }


    // ==========================================
    // 情况 2：当前帧没有检测到装甲板
    // ==========================================

    detected_frames_ = 0;


    // --------------------------------------
    // 2.1 此前存在目标
    // --------------------------------------

    if (had_target_)
    {
        lost_frames_++;

        if (
            lost_frames_ <=
            temporary_lost_threshold_
        )
        {
            // 短暂丢失：
            // 保留历史目标状态，但当前帧目标无效
            result.status =
                TargetStatus::TEMP_LOST;
        }
        else
        {
            // 连续丢失超过阈值：
            // 正式进入无目标状态
            result.status =
                TargetStatus::NO_TARGET;

            had_target_ = false;
            lost_frames_ = 0;
        }
    }


    // --------------------------------------
    // 2.2 此前也不存在目标
    // --------------------------------------

    else
    {
        result.status =
            TargetStatus::NO_TARGET;
    }


    // 当前帧没有真实检测结果，
    // 因此不进行位姿解算
    result.valid = false;

    return result;
}