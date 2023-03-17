
#ifndef UDP_H
#define UDP_H

#include <string>
#include <vector>

class UDP
{
public:
    UDP(std::string& target_udp_ip, int target_udp_port, int pc_udp_port);
    ~UDP();

    bool isOpen(void);
    void SendPacket(std::vector<uint8_t> send_packet);
    int RecvPacket(std::vector<uint8_t> &send_packet);

private:
    void *udp_;
};

#endif