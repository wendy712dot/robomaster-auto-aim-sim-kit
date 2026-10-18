#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <array>
#include <cstdint>

#include "target_selector.hpp"


// 一帧模拟 CAN 报文
struct CanFrame
{
    uint32_t id = 0x301;
    std::array<uint8_t, 8> data{};
};


// 接收端解析后的数据
struct CommunicationData
{
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;

    TargetStatus status = TargetStatus::NO_TARGET;

    bool checksum_valid = false;
};


class Communication
{
public:
    // 将视觉结果编码为 8 字节报文
    CanFrame encode(
        double yaw,
        double pitch,
        double distance,
        TargetStatus status
    ) const;

    // 将 8 字节报文解析回原始数据
    CommunicationData decode(
        const CanFrame& frame
    ) const;

    // 计算前 7 字节的校验和
    uint8_t calculateChecksum(
        const std::array<uint8_t, 8>& data
    ) const;
};


#endif