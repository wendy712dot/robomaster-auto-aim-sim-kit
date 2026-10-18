#include "udp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>


UdpSender::UdpSender(
    const std::string& ip,
    int port
)
    : socket_fd_(-1),
      ip_(ip),
      port_(port)
{
    socket_fd_ =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );
}


UdpSender::~UdpSender()
{
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
    }
}


bool UdpSender::isValid() const
{
    return socket_fd_ >= 0;
}


bool UdpSender::send(
    const CanFrame& frame
)
{
    if (!isValid())
    {
        return false;
    }


    sockaddr_in receiver_addr{};

    receiver_addr.sin_family =
        AF_INET;

    receiver_addr.sin_port =
        htons(port_);

    receiver_addr.sin_addr.s_addr =
        inet_addr(
            ip_.c_str()
        );


    ssize_t sent_bytes =
        sendto(
            socket_fd_,
            frame.data.data(),
            frame.data.size(),
            0,
            reinterpret_cast<sockaddr*>(
                &receiver_addr
            ),
            sizeof(receiver_addr)
        );


    return sent_bytes ==
           static_cast<ssize_t>(
               frame.data.size()
           );
}