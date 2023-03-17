
#include "gl_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>
#include <mutex>

#include "serial/serial.h"
#include "udp/udp.h"

namespace SOSLAB
{
    enum PAYLOAD_DELIMETERS
    {
        PS1 = 0xC3,
        PS2 = 0x51,
        PS3 = 0xA1,
        PS4 = 0xF8,
        PE = 0xC2
    };

    enum PAYLOAD_SM_PROPS
    {
        SM_SET = 0,
        SM_GET = 1,
        SM_STREAM = 2,
        SM_ERROR = 255,
    };

    enum PAYLOAD_DIRECTION
    {
        BI_PC2GL = 0x21,
        BI_GL2PC = 0x12,
    };

    enum PAYLOAD_STATE
    {
        STATE_INIT = 0,
        STATE_PS1 = 1,
        STATE_PS2 = 2,
        STATE_PS3 = 3,
        STATE_PS4 = 4,
        STATE_TL = 5,
        STATE_PAYLOAD = 6,
        STATE_CS = 7,
    };

    enum COMM_TYPE
    {
        COMM_SERIAL = 1,
        COMM_UDP = 2,
    };

    class ObjectForComm
    {
    public:
        uint8_t comm_type_;
        
        int recv_state_;
        uint8_t read_cs_;
        uint8_t write_cs_;

        std::vector<uint8_t> send_packet_;
        std::vector<uint8_t> recv_packet_;

    public:
        ObjectForComm()
            : recv_state_(STATE_INIT)
            , read_cs_(0x00)
            , write_cs_(0x00)
        {
        }

        ~ObjectForComm()
        {
        }

        void read_cs_update(uint8_t data)
        {
            read_cs_ = read_cs_ ^ (data & 0xff);
        }

        uint8_t read_cs_get()
        {
            return read_cs_ & 0xff;
        }

        void read_cs_clear()
        {
            read_cs_ = 0x00;
        }

        void write_cs_update(uint8_t data)
        {
            write_cs_ = write_cs_ ^ (data & 0xff);
        }

        uint8_t write_cs_get()
        {
            return write_cs_ & 0xff;
        }

        void write_cs_clear()
        {
            write_cs_ = 0x00;
        }

        void write(uint8_t data)
        {
            send_packet_.push_back(data);
            write_cs_update(data);
        }

        void write_PS()
        {
            std::vector<uint8_t> PS = {PS1, PS2, PS3, PS4};

            for (auto &i : PS)
                write(i);
        }

        void RecvPacketClear(void)
        {
            read_cs_clear();
            recv_state_ = STATE_INIT;
            recv_packet_.clear();
        }

        void CheckPS(uint8_t data)
        {
            if (recv_state_ == STATE_INIT && data == PS1)
            {
                RecvPacketClear();
                read_cs_update(data);
                recv_state_ = STATE_PS1;
                return;
            }
            else if (recv_state_ == STATE_PS1 && data == PS2)
            {
                read_cs_update(data);
                recv_state_ = STATE_PS2;
                return;
            }
            else if (recv_state_ == STATE_PS2 && data == PS3)
            {
                read_cs_update(data);
                recv_state_ = STATE_PS3;
                return;
            }
            else if (recv_state_ == STATE_PS3 && data == PS4)
            {
                read_cs_update(data);
                recv_state_ = STATE_PS4;
                return;
            }

            recv_state_ = STATE_INIT;
        }
    };

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

    /*
    Implementation for the GL class
    */
    class impl_GL : private ObjectForComm
    {
        serial::Serial *serial_port_;
        UDP *udp_;

        std::mutex mut_;
        std::vector<uint8_t> serial_num_;
        std::vector<std::vector<uint8_t>> lidar_data_;
        GL::framedata_t frame_data_in_;

        bool thread_running_;
        std::thread *th_;

    public:
        impl_GL(std::string &gl_udp_ip, int gl_udp_port, int pc_udp_port)
            : serial_port_(nullptr)
            , udp_(new UDP(gl_udp_ip, gl_udp_port, pc_udp_port))
            , thread_running_(true)
            , th_(nullptr)
        {
            if (udp_->isOpen())
            {
                comm_type_ = COMM_UDP;
                std::cout << "GL UDP is opened." << std::endl;
                th_ = new std::thread(&impl_GL::ThreadCallBack, this);
            }
            else
            {
                throw exception_msg("[ERROR] GL UDP is not opened.", -1);
            }
        }

        impl_GL(std::string &gl_serial_name, uint32_t gl_serial_baudrate)
            : serial_port_(new serial::Serial(gl_serial_name, gl_serial_baudrate, serial::Timeout::simpleTimeout(10)))
            , udp_(nullptr)
            , thread_running_(true)
            , th_(nullptr)
        {
            if (serial_port_->isOpen())
            {
                comm_type_ = COMM_SERIAL;
                std::cout << "GL Serial is opened." << std::endl;
                th_ = new std::thread(&impl_GL::ThreadCallBack, this);
            }
            else
            {
                throw exception_msg("[ERROR] GL Serial is not opened.", -1);
            }
        }

        ~impl_GL()
        {
            SetFrameDataEnable(false);
            thread_running_ = false;
            if (nullptr != th_)
            {
                th_->join();
            }

            if (ObjectForComm::comm_type_ == COMM_SERIAL)
            {
                serial_port_->close();
                delete serial_port_;
                std::cout << "Serial END" << std::endl;
            }
            else if (ObjectForComm::comm_type_ == COMM_UDP)
            {
                delete udp_;
            }
        }

        /* Read GL Conditions */
        std::string GetSerialNum(void)
        {
            uint8_t PI = 0;
            uint8_t PL = 1;
            uint8_t SM = SM_GET;
            uint8_t CAT0 = 0x02;
            uint8_t CAT1 = 0x0A;
            std::vector<uint8_t> DTn = {1};

            mut_.lock();
            serial_num_.clear();
            mut_.unlock();
            for (size_t i = 0; i < 50; i++)
            {
                WritePacket(PI, PL, SM, CAT0, CAT1, DTn);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                if (serial_num_.size() > 0)
                {
                    mut_.lock();
                    std::string out_str(serial_num_.begin(), serial_num_.end());
                    mut_.unlock();
                    return out_str;
                }
            }

            std::string out_str = "[ERROR] Serial Number is not received.";
            return out_str;
        }

        void ReadFrameData(GL::framedata_t &frame_data, bool filter_on = true)
        {
            mut_.lock();
            frame_data = frame_data_in_;
            frame_data_in_.angle.clear();
            frame_data_in_.distance.clear();
            frame_data_in_.pulse_width.clear();
            mut_.unlock();

            if (filter_on == true)
            {
                for (int i = 0; i < (int)frame_data.distance.size() - 1; i++)
                {
                    if (frame_data.distance[i] > 0.0 && frame_data.distance[i + 1] > 0.0)
                    {
                        double diff = (frame_data.distance[i] - frame_data.distance[i + 1]) / 2.0;
                        if (diff > 0.01 * frame_data.distance[i])
                            frame_data.distance[i] = 0.0;
                        if (diff < -0.01 * frame_data.distance[i])
                            frame_data.distance[i] = 0.0;
                    }
                }
            }
        }

        /* Set GL Conditions */
        void SetFrameDataEnable(bool framedata_enable)
        {
            uint8_t PI = 0;
            uint8_t PL = 1;
            uint8_t SM = SM_SET;
            uint8_t CAT0 = 0x1;
            uint8_t CAT1 = 0x3;
            std::vector<uint8_t> DTn = {framedata_enable};

            WritePacket(PI, PL, SM, CAT0, CAT1, DTn);
        }

    private:
        void ThreadCallBack(void)
        {
            RecvPacketClear();

            while (thread_running_ == true)
            {
                if (ObjectForComm::comm_type_ == COMM_SERIAL)
                {
                    uint8_t data;
                    while (thread_running_ == true && serial_port_->read(&data, 1) == 1)
                        AddPacketElement(data);
                }
                else if (ObjectForComm::comm_type_ == COMM_UDP)
                {
                    std::vector<uint8_t> recv_packet(2000);
                    int recv_len = udp_->RecvPacket(recv_packet);

                    for (int i = 0; i < recv_len; i++)
                        AddPacketElement(recv_packet[i]);
                }
            }
        }

        void SendPacket(std::vector<uint8_t> &send_packet)
        {
            for (auto &i : send_packet)
                serial_port_->write(&i, 1);
        }

        void WritePacket(uint8_t PI, uint8_t PL, uint8_t SM, uint8_t CAT0, uint8_t CAT1, const std::vector<uint8_t> &DTn)
        {
            send_packet_.clear();
            write_cs_clear();

            write_PS();

            uint16_t TL = DTn.size() + 14;
            uint8_t buff = TL & 0xff;
            write(buff);
            buff = (TL >> 8) & 0xff;
            write(buff);

            write(PI);
            write(PL);
            write(SM);
            write(BI_PC2GL);
            write(CAT0);
            write(CAT1);

            for (auto i : DTn)
                write(i);

            write(PE);
            write(write_cs_get());

            if (ObjectForComm::comm_type_ == COMM_SERIAL)
                SendPacket(send_packet_);
            else if (ObjectForComm::comm_type_ == COMM_UDP)
                udp_->SendPacket(send_packet_);
        }

        void FrameData(const std::vector<uint8_t> &recv_data, uint8_t PI, uint8_t PL, uint8_t SM)
        {
            if (SM != SM_STREAM || recv_data.size() == 0)
                return;

            if (PI == 0)
            {
                lidar_data_.clear();
                lidar_data_.push_back(recv_data);
            }
            else if (PI == lidar_data_.size())
            {
                lidar_data_.push_back(recv_data);
            }
            else
            {
                lidar_data_.clear();
                return;
            }

            if (lidar_data_.size() == PL)
            {
                if (lidar_data_[0].size() < 3)
                {
                    lidar_data_.clear();
                    return;
                }

                std::vector<uint8_t> data;
                for (int i = 0; i < lidar_data_.size(); i++)
                    std::copy(lidar_data_[i].begin(), lidar_data_[i].end(), std::back_inserter(data));

                uint16_t frame_data_size = data[0] & 0xff;
                frame_data_size |= ((uint16_t)(data[1] & 0xff)) << 8;

                if (data.size() != (frame_data_size * 4 + 22))
                {
                    lidar_data_.clear();
                    return;
                }

                GL::framedata_t frame_data;
                frame_data.distance.resize(frame_data_size);
                frame_data.pulse_width.resize(frame_data_size);
                frame_data.angle.resize(frame_data_size);
                for (size_t i = 0; i < frame_data_size; i++)
                {
                    uint16_t distance = data[i * 4 + 2] & 0xff;
                    distance |= ((uint16_t)(data[i * 4 + 3] & 0xff)) << 8;

                    uint16_t pulse_width = data[i * 4 + 4] & 0xff;
                    pulse_width |= ((uint16_t)(data[i * 4 + 5] & 0xff)) << 8;

                    if(distance>30000) distance = 0;
                    frame_data.distance[i] = distance / 1000.0;
                    frame_data.pulse_width[i] = pulse_width;
                    frame_data.angle[i] = i * 180.0 / (frame_data_size - 1) * 3.141592 / 180.0;
                }

                mut_.lock();
                frame_data_in_ = frame_data;
                mut_.unlock();
                lidar_data_.clear();
            }
        }

        void SerialNum(const std::vector<uint8_t> &recv_data, uint8_t PI, uint8_t PL, uint8_t SM)
        {
            if (SM != SM_GET || recv_data.size() == 0)
                return;

            mut_.lock();
            serial_num_ = recv_data;
            mut_.unlock();
        }

        void ParsingData(const std::vector<uint8_t> &recv_data, uint8_t PI, uint8_t PL, uint8_t SM, uint8_t BI, uint8_t CAT0, uint8_t CAT1)
        {
            // std::cout << std::endl;
            // std::cout << "Recv Data" << std::endl;
            // std::cout << "PI = " << (int)PI << std::endl;
            // std::cout << "PL = " << (int)PL << std::endl;
            // std::cout << "SM = " << (int)SM << std::endl;
            // std::cout << "BI = " << (int)BI << std::endl;
            // std::cout << "CAT0 = " << (int)CAT0 << std::endl;
            // std::cout << "CAT1 = " << (int)CAT1 << std::endl;
            // std::cout << "DTL = " << recv_data.size() << std::endl;

            if (BI != BI_GL2PC)
                return;

            if (CAT0 == 0x01 && CAT1 == 0x02)
                FrameData(recv_data, PI, PL, SM);
            else if (CAT0 == 0x02 && CAT1 == 0x0A)
                SerialNum(recv_data, PI, PL, SM);
        }

        void ParsingPayload(const std::vector<uint8_t> &recv_packet)
        {
            std::vector<uint8_t> recv_data;

            uint16_t TL = recv_packet[0] & 0xff;
            TL |= ((uint16_t)recv_packet[1]) << 8;

            uint8_t PI = recv_packet[2] & 0xff;
            uint8_t PL = recv_packet[3] & 0xff;
            uint8_t SM = recv_packet[4] & 0xff;
            uint8_t BI = recv_packet[5] & 0xff;
            uint8_t CAT0 = recv_packet[6] & 0xff;
            uint8_t CAT1 = recv_packet[7] & 0xff;

            uint16_t DTL = TL - 14;

            recv_data.resize(DTL);
            for (int i = 0; i < DTL; i++)
            {
                recv_data[i] = recv_packet[8 + i];
            }

            ParsingData(recv_data, PI, PL, SM, BI, CAT0, CAT1);
        }

        void AddPacketElement(uint8_t data)
        {
            if (recv_state_ == STATE_INIT || recv_state_ == STATE_PS1 || recv_state_ == STATE_PS2 || recv_state_ == STATE_PS3)
            {
                CheckPS(data);
            }
            else if (recv_state_ == STATE_PS4)
            {
                recv_state_ = STATE_TL;
                recv_packet_.push_back(data);
                read_cs_update(data);
            }
            else if (recv_state_ == STATE_TL)
            {
                recv_state_ = STATE_PAYLOAD;
                recv_packet_.push_back(data);
                read_cs_update(data);
            }
            else if (recv_state_ == STATE_PAYLOAD)
            {
                if (recv_packet_.size() >= 8)
                {
                    uint16_t recv_TL;
                    recv_TL = recv_packet_[0] & 0xff;
                    recv_TL |= ((uint16_t)recv_packet_[1]) << 8;

                    if (recv_packet_.size() == (recv_TL - 6))
                    {
                        if (data == PE)
                        {
                            recv_state_ = STATE_CS;
                            read_cs_update(data);
                        }
                        else
                            recv_state_ = STATE_INIT;
                    }
                    else
                    {
                        recv_packet_.push_back(data);
                        read_cs_update(data);
                    }
                }
                else
                {
                    recv_packet_.push_back(data);
                    read_cs_update(data);
                }
            }
            else if (recv_state_ == STATE_CS)
            {
                if (data == read_cs_get())
                    ParsingPayload(recv_packet_);

                RecvPacketClear();
                return;
            }

            if (recv_state_ == STATE_INIT)
            {
                std::vector<uint8_t> packet = recv_packet_;
                RecvPacketClear();
                for (auto &v : packet)
                    AddPacketElement(v);
            }
        }
    };

    /*
    Library user interfaces (wrapping class of the impl_GL)
    */
    GL::GL(std::string &gl_udp_ip, int gl_udp_port, int pc_udp_port)
        : gl_(new impl_GL(gl_udp_ip, gl_udp_port, pc_udp_port))
    {
        // nothing to do.
    }

    GL::GL(std::string &gl_serial_name, uint32_t gl_serial_baudrate)
        : gl_(new impl_GL(gl_serial_name, gl_serial_baudrate))
    {
        // nothing to do.
    }

    GL::~GL()
    {
        delete static_cast<impl_GL *>(gl_);
    }

    std::string GL::GetSerialNum(void)
    {
        return static_cast<impl_GL *>(gl_)->GetSerialNum();
    }

    void GL::ReadFrameData(GL::framedata_t &frame_data, bool filter_on)
    {
        static_cast<impl_GL *>(gl_)->ReadFrameData(frame_data, filter_on);
    }

    void GL::SetFrameDataEnable(bool framedata_enable)
    {
        static_cast<impl_GL *>(gl_)->SetFrameDataEnable(framedata_enable);
    }

} /* namespace SOSLAB */
