#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <iomanip>
#include <atomic>
#include <thread>
#include <chrono>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#endif
class ModbusClient {
public:
    ModbusClient(const std::string &host, int port)
        : socket_(io_service_), host_(host), port_(port), transaction_id_(0) {
        setTimeout(5000);  // 기본 5초 타임아웃setReadTimeout(boost::posix_time::seconds(5));
        // setWriteTimeout(boost::posix_time::seconds(5));
    }

    virtual ~ModbusClient() {
        disconnect();
    }

    // void setReadTimeout(boost::posix_time::time_duration timeout) {
    //     // socket_.set_option(boost::asio::socket_base::receive_timeout(timeout));
    // socket_.set
    // }


    // void setWriteTimeout(boost::posix_time::time_duration timeout) {
    //     socket_.set_option(boost::asio::socket_base::send_timeout(timeout));
    // }
void setTimeout(int timeout_ms)
    {
        try {
            if (socket_.is_open()) {
                #ifdef _WIN32
                    // Windows
                    DWORD timeout = timeout_ms;
                    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, 
                             (const char*)&timeout, sizeof(timeout));
                    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_SNDTIMEO, 
                             (const char*)&timeout, sizeof(timeout));
                #else
                    // Linux/Unix
                    struct timeval tv;
                    tv.tv_sec = timeout_ms / 1000;
                    tv.tv_usec = (timeout_ms % 1000) * 1000;
                    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, 
                             (const char*)&tv, sizeof(tv));
                    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_SNDTIMEO, 
                             (const char*)&tv, sizeof(tv));
                #endif
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error setting timeout: " << e.what() << std::endl;
        }
    }
    bool connect(int timeout_seconds = 5) {
        try {
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::system::error_code ec;

            boost::asio::ip::tcp::resolver::query query(host_, std::to_string(port_));
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            
            // Connect with timeout
            boost::asio::deadline_timer timer(io_service_);
            timer.expires_from_now(boost::posix_time::seconds(timeout_seconds));
            
            bool timedOut = false;
            timer.async_wait([&](const boost::system::error_code& error) {
                if (!error) {
                    socket_.close();
                    timedOut = true;
                }
            });

            boost::asio::async_connect(socket_, endpoint_iterator,
                [&](const boost::system::error_code& error, 
                    boost::asio::ip::tcp::resolver::iterator) {
                    timer.cancel();
                });

            io_service_.run();
            
            if (timedOut || !socket_.is_open()) {
                std::cerr << "Connection timeout" << std::endl;
                return false;
            }

            std::cout << "Connected to " << host_ << ":" << port_ << std::endl;
            return true;
        }
        catch (std::exception &e) {
            std::cerr << "Connection failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool reconnect(int max_attempts = 3) {
        for(int i = 0; i < max_attempts; i++) {
            std::cout << "Reconnection attempt " << (i + 1) << "/" << max_attempts << std::endl;
            disconnect();
            if(connect()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return false;
    }

    void disconnect() {
        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
            socket_.close(ec);
        }
    }

    std::vector<int16_t> readHoldingRegisters(uint16_t startAddress, uint16_t quantity) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            uint16_t current_transaction = transaction_id_++;

            std::vector<uint8_t> request = {
                static_cast<uint8_t>(current_transaction >> 8), 
                static_cast<uint8_t>(current_transaction & 0xFF),
                0x00, 0x00,                                    
                0x00, 0x06,                                    
                0x01,                                          
                0x03,                                          
                static_cast<uint8_t>(startAddress >> 8), 
                static_cast<uint8_t>(startAddress & 0xFF),
                static_cast<uint8_t>(quantity >> 8), 
                static_cast<uint8_t>(quantity & 0xFF)
            };

            boost::asio::write(socket_, boost::asio::buffer(request));

            std::vector<uint8_t> response(9 + quantity * 2);  // 수정된 버퍼 크기
            size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(response));

            // Transaction ID 확인
            uint16_t received_transaction = (response[0] << 8) | response[1];
            if(current_transaction != received_transaction) {
                throw std::runtime_error("Transaction ID mismatch in readHoldingRegisters");
            }

            // 함수 코드 확인
            if(response[7] != 0x03) {
                if(response[7] == 0x83) {
                    throw std::runtime_error("Modbus exception received");
                }
                throw std::runtime_error("Invalid function code");
            }

            std::vector<int16_t> registers;
            registers.reserve(quantity);

            for (size_t i = 0; i < quantity; ++i) {
                int16_t value = static_cast<int16_t>(
                    (response[9 + i * 2] << 8) | response[10 + i * 2]
                );
                registers.push_back(value);
            }

            return registers;
        }
        catch(const std::exception& e) {
            std::cerr << "Error in readHoldingRegisters: " << e.what() << std::endl;
            throw;
        }
    }

    std::vector<int16_t> readInputRegisters(uint16_t startAddress, uint16_t quantity) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            uint16_t current_transaction = transaction_id_++;
            
            std::vector<uint8_t> request(12);
            request[0] = static_cast<uint8_t>(current_transaction >> 8);
            request[1] = static_cast<uint8_t>(current_transaction & 0xFF);
            request[2] = 0x00;
            request[3] = 0x00;
            request[4] = 0x00;
            request[5] = 0x06;
            request[6] = 0x01;
            request[7] = 0x04;
            request[8] = static_cast<uint8_t>(startAddress >> 8);
            request[9] = static_cast<uint8_t>(startAddress & 0xFF);
            request[10] = static_cast<uint8_t>(quantity >> 8);
            request[11] = static_cast<uint8_t>(quantity & 0xFF);

            // 요청 전송
            boost::asio::write(socket_, boost::asio::buffer(request));

            // 응답 수신
            std::vector<uint8_t> response(9 + quantity * 2);
            size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(response));

            // Transaction ID 확인 및 재시도 로직
            uint16_t received_transaction = (response[0] << 8) + response[1];
            if(current_transaction != received_transaction) {
                std::cerr << "Transaction ID mismatch. Expected: " << current_transaction 
                         << ", Got: " << received_transaction << std::endl;
                
                // 최대 3번까지 재시도
                for(int retry = 0; retry < 3; retry++) {
                    std::cout << "Retrying... Attempt " << (retry + 1) << std::endl;
                    
                    boost::asio::write(socket_, boost::asio::buffer(request));
                    boost::asio::read(socket_, boost::asio::buffer(response));
                    
                    received_transaction = (response[0] << 8) + response[1];
                    if(received_transaction == current_transaction) {
                        std::cout << "Success after retry" << std::endl;
                        break;
                    }
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                if(received_transaction != current_transaction) {
                    throw std::runtime_error("Transaction ID mismatch after retries");
                }
            }

            // 프로토콜 ID 확인
            if(response[2] != 0x00 || response[3] != 0x00) {
                throw std::runtime_error("Invalid protocol ID");
            }

            // 함수 코드 확인
            if(response[7] != 0x04) {
                if(response[7] == 0x84) {
                    throw std::runtime_error("Modbus exception received");
                }
                throw std::runtime_error("Invalid function code");
            }

            // 바이트 수 확인
            uint8_t byte_count = response[8];
            if(byte_count != quantity * 2) {
                throw std::runtime_error("Incorrect byte count");
            }

            std::vector<int16_t> registers;
            registers.reserve(quantity);

            for(size_t i = 0; i < quantity; ++i) {
                int16_t value = static_cast<int16_t>(
                    (response[9 + i * 2] << 8) | response[10 + i * 2]
                );
                registers.push_back(value);
                // std::cout << "Register[" << i << "] = " << value << std::endl;
            }

            return registers;
        }
        catch(const std::exception& e) {
            std::cerr << "Error in readInputRegisters: " << e.what() << std::endl;
            throw;
        }
    }

    void writeSingleCoil(uint16_t address, bool value) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            uint16_t current_transaction = transaction_id_++;

            std::vector<uint8_t> request = {
                static_cast<uint8_t>(current_transaction >> 8), 
                static_cast<uint8_t>(current_transaction & 0xFF),
                0x00, 0x00,                                    
                0x00, 0x06,                                    
                0x01,                                          
                0x05,                                          
                static_cast<uint8_t>(address >> 8), 
                static_cast<uint8_t>(address & 0xFF),
                static_cast<uint8_t>(value ? 0xFF : 0x00), 
                0x00                                          
            };

            boost::asio::write(socket_, boost::asio::buffer(request));
            
            // 응답 수신
            std::vector<uint8_t> response(12);
            boost::asio::read(socket_, boost::asio::buffer(response));

            // Transaction ID 확인
            uint16_t received_transaction = (response[0] << 8) | response[1];
            if(current_transaction != received_transaction) {
                throw std::runtime_error("Transaction ID mismatch");
            }

            // 함수 코드 확인
            if(response[7] != 0x05) {
                if(response[7] == 0x85) {
                    throw std::runtime_error("Modbus exception received");
                }
                throw std::runtime_error("Invalid function code");
            }
        }
        catch(const std::exception& e) {
            std::cerr << "Error in writeSingleCoil: " << e.what() << std::endl;
            throw;
        }
    }

    void writeMultiCoils(uint16_t address, const std::vector<bool> &values) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            uint16_t current_transaction = transaction_id_++;

            size_t num_bytes = (values.size() + 7) / 8;        
            std::vector<uint8_t> value_bytes(num_bytes, 0x00); 

            for (size_t i = 0; i < values.size(); i++) {
                if (values[i]) {
                    value_bytes[i / 8] |= 1 << (i % 8);
                }
            }

            std::vector<uint8_t> request = {
                static_cast<uint8_t>(current_transaction >> 8), 
                static_cast<uint8_t>(current_transaction & 0xFF),
                0x00, 0x00,                                    
                static_cast<uint8_t>((7 + num_bytes) >> 8), 
                static_cast<uint8_t>(7 + num_bytes & 0xFF),
                0x01,                                          
                0x0F,                                          
                static_cast<uint8_t>(address >> 8), 
                static_cast<uint8_t>(address & 0xFF),
                static_cast<uint8_t>(values.size() >> 8), 
                static_cast<uint8_t>(values.size() & 0xFF),
                static_cast<uint8_t>(num_bytes)               
            };

            request.insert(request.end(), value_bytes.begin(), value_bytes.end());

            boost::asio::write(socket_, boost::asio::buffer(request));
            
            // 응답 수신
            std::vector<uint8_t> response(12);
            boost::asio::read(socket_, boost::asio::buffer(response));

            // Transaction ID 확인
            uint16_t received_transaction = (response[0] << 8) | response[1];
            if(current_transaction != received_transaction) {
                throw std::runtime_error("Transaction ID mismatch");
            }

            // 함수 코드 확인
            if(response[7] != 0x0F) {
                if(response[7] == 0x8F) {
                    throw std::runtime_error("Modbus exception received");
                }
                throw std::runtime_error("Invalid function code");
            }
        }
        catch(const std::exception& e) {
            std::cerr << "Error in writeMultiCoils: " << e.what() << std::endl;
            throw;
        }
    }

    void writeSingleRegister(uint16_t address, uint16_t value) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            uint16_t current_transaction = transaction_id_++;

            std::vector<uint8_t> request = {
                static_cast<uint8_t>(current_transaction >> 8), 
                static_cast<uint8_t>(current_transaction & 0xFF),
                0x00, 0x00,                                    
                0x00, 0x06,                                    
                0x01,                                          
                0x06,                                          
                static_cast<uint8_t>(address >> 8), 
                static_cast<uint8_t>(address & 0xFF),
                static_cast<uint8_t>(value >> 8), 
                static_cast<uint8_t>(value & 0xFF)
            };

            boost::asio::write(socket_, boost::asio::buffer(request));
            
            // 응답 수신
           std::vector<uint8_t> response(12);
           boost::asio::read(socket_, boost::asio::buffer(response));

           // Transaction ID 확인
           uint16_t received_transaction = (response[0] << 8) | response[1];
           if(current_transaction != received_transaction) {
               throw std::runtime_error("Transaction ID mismatch");
           }

           // 함수 코드 확인
           if(response[7] != 0x06) {
               if(response[7] == 0x86) {
                   throw std::runtime_error("Modbus exception received");
               }
               throw std::runtime_error("Invalid function code");
           }
       }
       catch(const std::exception& e) {
           std::cerr << "Error in writeSingleRegister: " << e.what() << std::endl;
           throw;
       }
   }

   void writeMultiRegisters(uint16_t address, const std::vector<uint16_t> &values) {
       try {
           std::lock_guard<std::mutex> lock(mutex_);
           uint16_t current_transaction = transaction_id_++;

           size_t values_bytes = values.size() * 2;

           std::vector<uint8_t> request = {
               static_cast<uint8_t>(current_transaction >> 8), 
               static_cast<uint8_t>(current_transaction & 0xFF),
               0x00, 0x00,                                    
               static_cast<uint8_t>((7 + values_bytes) >> 8), 
               static_cast<uint8_t>(7 + values_bytes & 0xFF),
               0x01,                                          
               0x10,                                          
               static_cast<uint8_t>(address >> 8), 
               static_cast<uint8_t>(address & 0xFF),
               static_cast<uint8_t>(values.size() >> 8), 
               static_cast<uint8_t>(values.size() & 0xFF),
               static_cast<uint8_t>(values_bytes)            
           };

           for (const auto &value : values) {
               request.push_back(static_cast<uint8_t>(value >> 8));
               request.push_back(static_cast<uint8_t>(value & 0xFF));
           }

           boost::asio::write(socket_, boost::asio::buffer(request));
           
           // 응답 수신
           std::vector<uint8_t> response(12);
           boost::asio::read(socket_, boost::asio::buffer(response));

           // Transaction ID 확인
           uint16_t received_transaction = (response[0] << 8) | response[1];
           if(current_transaction != received_transaction) {
               throw std::runtime_error("Transaction ID mismatch");
           }

           // 함수 코드 확인
           if(response[7] != 0x10) {
               if(response[7] == 0x90) {
                   throw std::runtime_error("Modbus exception received");
               }
               throw std::runtime_error("Invalid function code");
           }
       }
       catch(const std::exception& e) {
           std::cerr << "Error in writeMultiRegisters: " << e.what() << std::endl;
           throw;
       }
   }

   // 유틸리티 함수 추가
   bool isConnected() const {
       return socket_.is_open();
   }

   void clearBuffers() {
       if(socket_.is_open()) {
           boost::system::error_code ec;
           socket_.cancel(ec);
           
           // 수신 버퍼 비우기
           std::vector<uint8_t> buffer(1024);
           while(socket_.available() > 0) {
               socket_.read_some(boost::asio::buffer(buffer), ec);
           }
       }
   }

private:
   boost::asio::io_service io_service_;
   boost::asio::ip::tcp::socket socket_;
   std::string host_;
   uint16_t port_;
   std::atomic<uint16_t> transaction_id_;
   std::mutex mutex_;

   // 헬퍼 함수 (필요한 경우 사용)
   void handleModbusException(uint8_t exceptionCode) {
       std::string error;
       switch(exceptionCode) {
           case 0x01: error = "Illegal function"; break;
           case 0x02: error = "Illegal data address"; break;
           case 0x03: error = "Illegal data value"; break;
           case 0x04: error = "Slave device failure"; break;
           case 0x05: error = "Acknowledge"; break;
           case 0x06: error = "Slave device busy"; break;
           case 0x08: error = "Memory parity error"; break;
           case 0x0A: error = "Gateway path unavailable"; break;
           case 0x0B: error = "Gateway target device failed to respond"; break;
           default: error = "Unknown error";
       }
       throw std::runtime_error("Modbus exception: " + error);
   }
};