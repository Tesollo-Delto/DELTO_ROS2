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

    
// 안정적인 패킷 수신 함수: 정확히 TOTAL_PACKET_SIZE만큼 읽을 때까지 대기
bool Communication::read_full_packet(boost::asio::ip::tcp::socket &socket, std::array<uint8_t, TOTAL_PACKET_SIZE> &buffer) {
    boost::system::error_code ec;
    std::size_t total_read = 0;
    while (total_read < TOTAL_PACKET_SIZE) {
        std::size_t bytes_read = socket.read_some(boost::asio::buffer(buffer.data() + total_read, TOTAL_PACKET_SIZE - total_read), ec);
        if (ec) {
            std::cerr << "Read error: " << ec.message() << std::endl;
            return false;
        }
        total_read += bytes_read;
    }
    return true;
}

// get_data 함수 예시
DeltoRecievedData Communication::get_data() {
    
    // 요청 패킷 전송
    //auto boost::asio::ip::tcp::socket& socket = socket_;   
    {
        std::array<uint8_t, 5> request = {0x01, 0x05, 0xEE, 0xD2, 0xDC};
        boost::system::error_code ec;
        socket_.write_some(boost::asio::buffer(request), ec);
        if (ec) {
            std::cerr << "Error sending request: " << ec.message() << std::endl;
            return DeltoRecievedData{};
        }
    }

    std::array<uint8_t, TOTAL_PACKET_SIZE> response;
    if (!read_full_packet(socket_, response)) {
        // 읽기 실패 시 빈 데이터 반환
        return DeltoRecievedData{};
    }

    // 헤더 검증
    uint8_t cmd = response[0];
    uint8_t length = response[1];
    if (cmd != EXPECTED_CMD || length != EXPECTED_LENGTH) {
        std::cerr << "Invalid header (CMD or LENGTH mismatch)" << std::endl;
        return DeltoRecievedData{};
    }

    // CRC 검증
    std::vector<uint8_t> data_for_crc(response.begin(), response.end() - CRC_SIZE);
    uint16_t recv_crc = (static_cast<uint16_t>(response[TOTAL_PACKET_SIZE - 1]) << 8) | response[TOTAL_PACKET_SIZE - 2];
    uint16_t calc_crc = calculate_crc16_arc(data_for_crc);
    if (recv_crc != calc_crc) {
        std::cerr << "CRC Mismatch. Received: 0x" << std::hex << recv_crc << " Calculated: 0x" << calc_crc << std::dec << std::endl;
        return DeltoRecievedData{};
    }

    // 모터 데이터 파싱
    DeltoRecievedData received_data;
    received_data.joint.resize(MOTOR_COUNT);
    received_data.current.resize(MOTOR_COUNT);

    // 모터 데이터 시작 오프셋: HEADER_SIZE=2
    // 총 12개 모터, 각 모터마다 [ID(1) + PosL(1) + PosH(1) + CurL(1) + CurH(1)] = 5바이트
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        size_t base = HEADER_SIZE + i * BYTES_PER_MOTOR;
        uint8_t motor_id = response[base]; // ID
        
        uint8_t posL = response[base + 1];
        uint8_t posH = response[base + 2];
        uint8_t curL = response[base + 3];
        uint8_t curH = response[base + 4];

        int16_t raw_position = combine_msg(posL, posH);
        int16_t raw_current  = combine_msg(curL, curH);

        // 위치는 rad로 환산 (조건에 따라 스케일 변경)
        double position_rad = raw_position * POSITION_SCALE;

        received_data.joint[i] = position_rad;
        received_data.current[i] = raw_current * CURRENT_SCALE;
    }

        //std::cout << "received_data.joint:" << received_data.joint[0] << std::endl;
    return received_data;
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