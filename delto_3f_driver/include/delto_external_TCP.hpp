#include <iostream>
#include <vector>
#include <cmath>
#include <boost/asio.hpp>
#include <array>
#include <cstdint>

static const uint8_t REFLECT_BIT_ORDER_TABLE[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0,
    0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8,
    0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4,
    0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC,
    0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2,
    0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA,
    0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6,
    0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE,
    0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1,
    0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9,
    0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5,
    0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED,
    0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
    0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
    0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7,
    0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF,
    0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

class DeltoRecievedData
{
    public:
    std::vector<double> joint;
    std::vector<double> current;    
};

using boost::asio::ip::tcp;
namespace asio = boost::asio;

namespace DeltoTCP 
{

    class Communication
    {
    public:
        Communication(const std::string& ip, int port);
        ~Communication();

        void connect();
        DeltoRecievedData get_data();
        void send_duty(std::vector<int>& duty);
    private:
        std::string ip_;
        int port_;
        asio::io_context io_context_;
        tcp::socket socket_;
        uint16_t calculate_crc16_arc(const std::vector<uint8_t>& data);
        uint16_t reflect(uint16_t byte);
        int16_t combine_msg(uint8_t data1, uint8_t data2);
    };

    Communication::Communication(const std::string& ip, int port)
        : ip_(ip), port_(port), socket_(io_context_) {}

    Communication::~Communication() 
    {
        socket_.close();
    }

    void Communication::connect()
    {
    tcp::resolver resolver(io_context_);
    boost::system::error_code ec;
    boost::asio::connect(socket_, resolver.resolve(ip_, std::to_string(port_)), ec);
    
    if (ec) 
        {
            std::cerr << "Could not connect: " << ec.message() << std::endl;

            return;
        }

        std::cout << "Connected to Delto Gripper" << std::endl;
    }

    DeltoRecievedData Communication::get_data() 
    {
    std::array<uint8_t, 5> request = {0x01, 0x05, 0xEE, 0xD2, 0xDC};
    boost::system::error_code ec;
    socket_.write_some(boost::asio::buffer(request), ec);

    std::array<uint8_t, 1024> response;
    size_t len = socket_.read_some(boost::asio::buffer(response), ec);

    DeltoRecievedData recived_data;
    recived_data.current.resize(12);
    recived_data.joint.resize(12);

    if (ec || len <= 59)
    {
        std::cerr << "Error reading response: " << ec.message() << std::endl;
        
            recived_data.current.resize(1);
            recived_data.joint.resize(1);
        
        return recived_data;
    }

    
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)

        {
            int index = 3 + i * 20 + j * 5;

            int16_t raw_joint = combine_msg(response[index], response[index + 1]);
            int16_t raw_current = combine_msg(response[index + 2], response[index + 3]);

            recived_data.joint[i * 4 + j] = (raw_joint / 1800.0) * M_PI;
            recived_data.current[i * 4 + j] = raw_current;
        }
    }

    return recived_data;
    }

void Communication::send_duty(std::vector<int>& duty)
 {
    
    std::vector<uint8_t> tcp_data_send = {0x03, 0x28,
                                          0x01, 0x00, 0x00, //motor Num, duty1, duty2
                                          0x02, 0x00, 0x00,
                                          0x03, 0x00, 0x00, 
                                          0x04, 0x00, 0x00,
                                          0x05, 0x00, 0x00, 
                                          0x06, 0x00, 0x00, 
                                          0x07, 0x00, 0x00,
                                          0x08, 0x00, 0x00,
                                          0x09, 0x00, 0x00,
                                          0x0A, 0x00, 0x00,
                                          0x0B, 0x00, 0x00,
                                          0x0C, 0x00, 0x00};

    for (size_t i = 0; i < 12; ++i) 
    {
        tcp_data_send[3 + i * 3] = (duty[i] >> 8) & 0xFF; //상위바이트
        tcp_data_send[4 + i * 3] = (duty[i] ) & 0xFF; //하위바이트
    }

    // Calculate CRC
    uint16_t crc = calculate_crc16_arc(tcp_data_send);

    
    tcp_data_send.push_back((crc) & 0xFF); //crc_l
    tcp_data_send.push_back((crc >>8)& 0xFF); //crc_h

    boost::system::error_code ec;

    socket_.write_some(boost::asio::buffer(tcp_data_send), ec);
    
    // for (auto byte:tcp_data_send)
    // {
    //     // std::cout << std::hex << int(byte) << " ";
    // }
    // std::cout << std::endl;
    
    // if (ec)
    //     {
    //         std::cerr << "Error sending duty command: " << ec.message() << std::endl;
    //     }
    }

    int16_t Communication::combine_msg(uint8_t data1, uint8_t data2) {
    // int16_t combined = static_cast<int16_t>((data1<< 8) | data2);
    int16_t combined = static_cast<int16_t>((data1 << 8) | (data2));
    
    if (combined >= 0x8000) 
    {
        combined -= 0x10000;
    }

    return combined;
}

    uint16_t Communication::calculate_crc16_arc(const std::vector<uint8_t>& data)
    {
    
        uint16_t crc = 0x0000;
        
        for (size_t i = 0; i < data.size(); i++) 
        {
            uint8_t byte = REFLECT_BIT_ORDER_TABLE[data[i]];
            crc ^= (byte << 8);
            for (int j = 0; j < 8; j++) 
            {
                crc = crc & 0x8000 ? (crc << 1) ^ 0x8005 : crc << 1;
            }
        }

        crc = reflect(crc);
        
        crc = (crc ^ 0x0000) & 0xFFFF;

        return crc;
    }

    uint16_t Communication::reflect(uint16_t value)
      {
        uint16_t reflected = 0;
        for (int i = 0; i < 16; i++) {
            if (value & 0x01)
                reflected |= (1 << ((16 - 1) - i));
            value = (value >> 1);
        }
        return reflected;
    }
}// namespace DeltoTCP