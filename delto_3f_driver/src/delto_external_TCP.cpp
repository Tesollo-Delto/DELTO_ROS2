#include "delto_external_TCP.hpp"


namespace DeltoTCP
{
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
    

    //print duty
    // for (auto d: duty)
    // {
    //        std::cout << d << " ";
    // }
    // std::cout << std::endl;

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
} // namespace DeltoTCP