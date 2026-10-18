#include <iostream>
#include <iomanip>

#include "communication.hpp"


int main()
{
    Communication communication;

    double yaw = 4.23;
    double pitch = -1.57;
    double distance = 600.0;

    TargetStatus status =
        TargetStatus::TRACKING;


    CanFrame frame =
        communication.encode(
            yaw,
            pitch,
            distance,
            status
        );


    std::cout
        << "CAN ID: 0x"
        << std::hex
        << std::uppercase
        << frame.id
        << std::endl;


    std::cout
        << "TX Data: ";

    for (uint8_t byte : frame.data)
    {
        std::cout
            << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(byte)
            << " ";
    }

    std::cout
        << std::dec
        << std::endl;


    CommunicationData received =
        communication.decode(frame);


    std::cout
        << std::fixed
        << std::setprecision(2);

    std::cout
        << "RX yaw: "
        << received.yaw
        << " deg"
        << std::endl;

    std::cout
        << "RX pitch: "
        << received.pitch
        << " deg"
        << std::endl;

    std::cout
        << "RX distance: "
        << received.distance
        << " mm"
        << std::endl;

    std::cout
        << "RX status: "
        << static_cast<int>(received.status)
        << std::endl;

    std::cout
        << "Checksum: "
        << (
            received.checksum_valid
            ? "OK"
            : "ERROR"
        )
        << std::endl;

    std::cout
    << "\n--- Corrupted Frame Test ---"
    << std::endl;


    // 复制一份正常报文
    CanFrame corrupted_frame = frame;


    // 故意修改第 0 个字节，模拟通信过程中数据出错
    corrupted_frame.data[0] ^= 0x01;


    std::cout
        << "Corrupted Data: ";

    for (uint8_t byte : corrupted_frame.data)
    {
        std::cout
            << std::hex
            << std::uppercase
            << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(byte)
            << " ";
    }

    std::cout
        << std::dec
        << std::endl;


    CommunicationData corrupted_received =
        communication.decode(corrupted_frame);


    std::cout
        << "Checksum: "
        << (
            corrupted_received.checksum_valid
            ? "OK"
            : "ERROR"
        )
        << std::endl;

    return 0;
}