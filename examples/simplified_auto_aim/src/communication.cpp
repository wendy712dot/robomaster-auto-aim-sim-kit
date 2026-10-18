#include "communication.hpp"

#include <cmath>


uint8_t Communication::calculateChecksum(
    const std::array<uint8_t, 8>& data
) const
{
    uint16_t sum = 0;

    for (int i = 0; i < 7; ++i)
    {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF);
}


CanFrame Communication::encode(
    double yaw,
    double pitch,
    double distance,
    TargetStatus status
) const
{
    CanFrame frame;

    // 角度保留 0.01°，距离取整为 mm
    int16_t yaw_value =
        static_cast<int16_t>(
            std::round(yaw * 100.0)
        );

    int16_t pitch_value =
        static_cast<int16_t>(
            std::round(pitch * 100.0)
        );

    uint16_t distance_value =
        static_cast<uint16_t>(
            std::round(distance)
        );

    uint8_t status_value =
        static_cast<uint8_t>(status);


    // 小端序写入 yaw
    frame.data[0] =
        static_cast<uint8_t>(
            yaw_value & 0xFF
        );

    frame.data[1] =
        static_cast<uint8_t>(
            (yaw_value >> 8) & 0xFF
        );


    // 小端序写入 pitch
    frame.data[2] =
        static_cast<uint8_t>(
            pitch_value & 0xFF
        );

    frame.data[3] =
        static_cast<uint8_t>(
            (pitch_value >> 8) & 0xFF
        );


    // 小端序写入 distance
    frame.data[4] =
        static_cast<uint8_t>(
            distance_value & 0xFF
        );

    frame.data[5] =
        static_cast<uint8_t>(
            (distance_value >> 8) & 0xFF
        );


    // 目标状态
    frame.data[6] = status_value;


    // 前 7 字节校验和
    frame.data[7] =
        calculateChecksum(frame.data);


    return frame;
}

CommunicationData Communication::decode(
    const CanFrame& frame
) const
{
    CommunicationData result;

    // 先检查校验和
    uint8_t expected_checksum =
        calculateChecksum(frame.data);

    result.checksum_valid =
        (expected_checksum == frame.data[7]);

    // 小端序还原 yaw
    uint16_t yaw_raw =
        static_cast<uint16_t>(frame.data[0]) |
        (
            static_cast<uint16_t>(frame.data[1]) << 8
        );

    int16_t yaw_value =
        static_cast<int16_t>(yaw_raw);


    // 小端序还原 pitch
    uint16_t pitch_raw =
        static_cast<uint16_t>(frame.data[2]) |
        (
            static_cast<uint16_t>(frame.data[3]) << 8
        );

    int16_t pitch_value =
        static_cast<int16_t>(pitch_raw);


    // 小端序还原 distance
    uint16_t distance_value =
        static_cast<uint16_t>(frame.data[4]) |
        (
            static_cast<uint16_t>(frame.data[5]) << 8
        );


    // 状态
    uint8_t status_value =
        frame.data[6];


    // 恢复原始量纲
    result.yaw =
        static_cast<double>(yaw_value) / 100.0;

    result.pitch =
        static_cast<double>(pitch_value) / 100.0;

    result.distance =
        static_cast<double>(distance_value);

    result.status =
        static_cast<TargetStatus>(status_value);


    return result;
}