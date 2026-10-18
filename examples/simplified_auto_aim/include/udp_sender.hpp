#ifndef UDP_SENDER_HPP
#define UDP_SENDER_HPP

#include <string>

#include "communication.hpp"


class UdpSender
{
public:
    UdpSender(
        const std::string& ip,
        int port
    );

    ~UdpSender();

    bool isValid() const;

    bool send(
        const CanFrame& frame
    );

private:
    int socket_fd_;
    std::string ip_;
    int port_;
};


#endif