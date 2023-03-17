#include "udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <mutex>

#ifdef _WIN32

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#pragma comment(lib, "ws2_32.lib")
#include <winsock2.h>
#include <WS2tcpip.h>

#else

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#endif

/*
exception with message
*/
class exception_msg : public std::exception
{
    std::string msg_;
    int32_t code_;

public:
    explicit exception_msg(const std::string &msg, const int32_t code)
        : msg_(msg), code_(code) {}

    char const *what() const noexcept override
    {
        return msg_.c_str();
    }

    int code() const noexcept
    {
        return code_;
    }
};

class Impl_UDP
{
    int sockfd;
    struct sockaddr_in target_addr, pc_addr;
    bool is_open = false;

public:
    Impl_UDP(std::string &target_udp_ip, int target_udp_port, int pc_udp_port)
    {

#if defined(_MSC_VER)

        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        {
            throw exception_msg("[ERROR] UDP is not initialized", -1);
        }

#endif
        
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {
            throw exception_msg("[ERROR] Socket creation failed", -1);
        }

        memset(&target_addr, 0, sizeof(target_addr));
        target_addr.sin_family = AF_INET;
        target_addr.sin_addr.s_addr = inet_addr(target_udp_ip.c_str());
        target_addr.sin_port = htons(target_udp_port);

        memset(&pc_addr, 0, sizeof(pc_addr));
        pc_addr.sin_family = AF_INET;
        pc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        pc_addr.sin_port = htons(pc_udp_port);

        if (bind(sockfd, (struct sockaddr *)&pc_addr, sizeof(pc_addr)) == -1)
        {
            throw exception_msg("[ERROR] Bind failed", -1);
        }

        double timeout_in_seconds = 1.0;
#ifdef _WIN32
        DWORD timeout = timeout_in_seconds * 1000;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof timeout);
#else
        struct timeval tv;
        tv.tv_sec = timeout_in_seconds;
        tv.tv_usec = 0;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
#endif

        std::cout << "Socket START [" << sockfd << "]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        is_open = true;
    }

    ~Impl_UDP()
    {
#if defined(_MSC_VER)

        closesocket(sockfd);
        WSACleanup();

#else

        close(sockfd);

#endif

        std::cout << "Socket END" << std::endl;
    }

    bool isOpen(void)
    {
        return is_open;
    }

    void SendPacket(std::vector<uint8_t> send_packet)
    {
        sendto(sockfd, (char *)&send_packet[0], send_packet.size(), 0, (const struct sockaddr *)&target_addr, sizeof(target_addr));
    }

    int RecvPacket(std::vector<uint8_t> &recv_packet)
    {
        int recv_len = recv(sockfd, (char *)&recv_packet[0], recv_packet.size(), 0);

        return recv_len;
    }
};

/*
User interfaces
*/
UDP::UDP(std::string &target_udp_ip, int target_udp_port, int pc_udp_port)
    : udp_(new Impl_UDP(target_udp_ip, target_udp_port, pc_udp_port))
{
    // nothing to do.
}

UDP::~UDP()
{
    delete static_cast<Impl_UDP *>(udp_);
}

bool UDP::isOpen(void)
{
    return static_cast<Impl_UDP *>(udp_)->isOpen();
}

void UDP::SendPacket(std::vector<uint8_t> send_packet)
{
    static_cast<Impl_UDP *>(udp_)->SendPacket(send_packet);
}

int UDP::RecvPacket(std::vector<uint8_t> &send_packet)
{
    return static_cast<Impl_UDP *>(udp_)->RecvPacket(send_packet);
}
