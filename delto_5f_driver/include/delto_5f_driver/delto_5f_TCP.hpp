#include "modbusTCP.hpp"
#include "delto_5f_enum.hpp"

#include <thread>
#include <mutex>

class Delto5F_TCP
{

public:
    Delto5F_TCP(std::string &ip, uint16_t port)
        : ip_(ip), port_(port)
    {
        ModbusClient_ = std::make_unique<ModbusClient>(ip, port);
    };

    virtual ~Delto5F_TCP() = default;

    void connect();

    std::vector<double> get_position_rad();
    std::vector<double> get_position_deg();
    std::vector<double> get_current();  
    
    bool set_postion_rad(std::vector<double> positions);
    bool set_postion_deg(std::vector<double> positions);


    std::vector<double> grasp();
    bool set_garsp_mode(int mode);



    private:
    std::unique_ptr<ModbusClient> ModbusClient_;
    std::string ip_;
    uint16_t port_;
    std::mutex mutex_;
};

void Delto5F_TCP::connect()
{
    ModbusClient_->connect();
}

std::vector<double> Delto5F_TCP::get_position_rad()
{
    // std::lock_guard<std::mutex> lock(mutex_);

    std::vector<uint16_t> positions;
    std::vector<int16_t> signed_positions;
    std::vector<double> positions_float;

    positions.reserve(MOTOR_NUM);
    positions_float.resize(MOTOR_NUM);
    signed_positions.resize(MOTOR_NUM);

    signed_positions = ModbusClient_->readInputRegisters(MOTOR1_CURRENT_POSITION, MOTOR_NUM);



    for (size_t i = 0; i < signed_positions.size(); ++i) 
    {
    positions_float[i] = signed_positions[i] * M_PI / 1800.0;
    }
    

    return positions_float;
}
std::vector<double> Delto5F_TCP::get_current()
{
    std::vector<int16_t> currents;
    std::vector<double> currents_float;
    currents.reserve(MOTOR_NUM);
    currents_float.resize(MOTOR_NUM);
    currents = ModbusClient_->readInputRegisters(MOTOR1_ELECTRICAL_CURRENT, MOTOR_NUM);

    for (size_t i = 0; i < currents.size(); ++i) 
    {
    currents_float[i] = currents[i] / 10.0;
    }

    return currents_float;
}
std::vector<double> Delto5F_TCP::get_position_deg()
{
    std::vector<int16_t> positions;
    std::vector<double> positions_float;
    positions.reserve(MOTOR_NUM);
    positions_float.resize(MOTOR_NUM);
    positions = ModbusClient_->readInputRegisters(MOTOR1_CURRENT_POSITION, MOTOR_NUM);

    for (size_t i = 0; i < positions.size(); ++i) 
    {
    positions_float[i] = positions[i] / 10.0;
    }

    return positions_float;
}

bool Delto5F_TCP::set_postion_rad(std::vector<double> positions)
{
    std::vector<uint16_t> position_int;
    position_int.reserve(MOTOR_NUM);

    for (auto& position: positions)
    {
        position_int.push_back(position * 1800 / M_PI);
    }

    ModbusClient_->writeMultiRegisters(MOTOR1_TARGET_POSITION, position_int);
}

bool Delto5F_TCP::set_postion_deg(std::vector<double> positions)
{
    std::vector<uint16_t> position_int;
    position_int.reserve(MOTOR_NUM);

    for (auto& position: positions)
    {
        position_int.push_back(position * 10);
    }

    ModbusClient_->writeMultiRegisters(MOTOR1_TARGET_POSITION, position_int);
}
