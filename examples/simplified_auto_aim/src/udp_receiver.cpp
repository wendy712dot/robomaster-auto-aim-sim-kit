#include <iostream>
#include <iomanip>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "communication.hpp"


int main()
{
    Communication communication;

    int socket_fd =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (socket_fd < 0)
    {
        std::cerr
            << "Failed to create socket."
            << std::endl;

        return -1;
    }


    sockaddr_in server_addr{};

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(9000);

    server_addr.sin_addr.s_addr =
        inet_addr("127.0.0.1");


    if (
        bind(
            socket_fd,
            reinterpret_cast<sockaddr*>(
                &server_addr
            ),
            sizeof(server_addr)
        ) < 0
    )
    {
        std::cerr
            << "Failed to bind socket."
            << std::endl;

        close(socket_fd);

        return -1;
    }


    std::cout
        << "UDP receiver listening on "
        << "127.0.0.1:9000"
        << std::endl;


    while (true)
    {
        CanFrame frame;

        sockaddr_in client_addr{};

        socklen_t client_addr_length =
            sizeof(client_addr);


        ssize_t received_bytes =
            recvfrom(
                socket_fd,
                frame.data.data(),
                frame.data.size(),
                0,
                reinterpret_cast<sockaddr*>(
                    &client_addr
                ),
                &client_addr_length
            );


        if (received_bytes < 0)
        {
            std::cerr
                << "Failed to receive data."
                << std::endl;

            continue;
        }


        if (received_bytes != 8)
        {
            std::cerr
                << "Invalid packet length: "
                << received_bytes
                << std::endl;

            continue;
        }


        CommunicationData data =
            communication.decode(frame);


        std::cout
            << "RX: ";

        for (uint8_t byte : frame.data)
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
            << " | "
            << std::fixed
            << std::setprecision(2)
            << "Yaw: "
            << data.yaw
            << " deg, "
            << "Pitch: "
            << data.pitch
            << " deg, "
            << "Distance: "
            << data.distance
            << " mm, "
            << "Status: ";

        switch (data.status)
        {
            case TargetStatus::NO_TARGET:
                std::cout << "NO_TARGET";
                break;

            case TargetStatus::DETECTED:
                std::cout << "DETECTED";
                break;

            case TargetStatus::TRACKING:
                std::cout << "TRACKING";
                break;

            case TargetStatus::TEMP_LOST:
                std::cout << "TEMP_LOST";
                break;
        }

        std::cout
            << ", Checksum: "
            << (
                data.checksum_valid
                ? "OK"
                : "ERROR"
            )
            << std::endl;
    }


    close(socket_fd);

    return 0;
}