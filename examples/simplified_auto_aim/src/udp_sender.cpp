#include <iostream>
#include <iomanip>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "communication.hpp"


int main()
{
    Communication communication;

    CanFrame frame =
        communication.encode(
            4.23,
            -1.57,
            600.0,
            TargetStatus::TRACKING
        );


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


    sockaddr_in receiver_addr{};

    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_port = htons(9000);

    receiver_addr.sin_addr.s_addr =
        inet_addr("127.0.0.1");


    ssize_t sent_bytes =
        sendto(
            socket_fd,
            frame.data.data(),
            frame.data.size(),
            0,
            reinterpret_cast<sockaddr*>(
                &receiver_addr
            ),
            sizeof(receiver_addr)
        );


    if (sent_bytes < 0)
    {
        std::cerr
            << "Failed to send data."
            << std::endl;

        close(socket_fd);

        return -1;
    }


    std::cout
        << "Sent "
        << sent_bytes
        << " bytes: ";

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
        << std::endl;


    close(socket_fd);

    return 0;
}