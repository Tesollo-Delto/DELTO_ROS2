#include "system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delto_3f_driver 
{
    
    hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::CallbackReturn::SUCCESS 
        != hardware_interface::SystemInterface::on_init(info))
        {
            return CallbackReturn::ERROR;
        }
        positions.resize(info_.joints.size(), 0.0);
        velocities.resize(info_.joints.size(), 0.0);
        
        position_commands.resize(info_.joints.size(), 0.0);

        delto_ip = info.hardware_parameters.at("delto_ip");
        delto_port = std::stoi(info.hardware_parameters.at("delto_port"));

        float k_p = std::stod(info.hardware_parameters.at("p_gain"));
        float k_d = std::stod(info.hardware_parameters.at("d_gain"));

        p_gain.resize(12, k_p);
        d_gain.resize(12, k_d);

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            
            if (joint.command_interfaces.size() != 1) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SystemInterface"), "Joint '%s' needs a command interface.",
                joint.name.c_str());
                return CallbackReturn::ERROR;
                }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SystemInterface"), "Joint '%s' needs a %s command interface.",
                joint.name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
            }

            if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY )) {
            RCLCPP_ERROR(
            rclcpp::get_logger("SystemInterface"),
            "Joint '%s' needs the following state interfaces in this order: %s, %s, %s.",
            joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
            }
    }
  
        delto_client = std::make_unique<DeltoTCP::Communication>(delto_ip, delto_port);
        m_init_thread = std::thread(&SystemInterface::init, this);
        m_init_thread.detach();


        return CallbackReturn::SUCCESS;
    }
hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Deactivating ...please wait...");
  
  return CallbackReturn::SUCCESS;
}

    std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 2);  // position, velocity

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands[i]));
    }

    return command_interfaces;
}


    SystemInterface::return_type SystemInterface::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
    {
    // We currently allow any combination of command interfaces.
    return return_type::OK;
    }

    SystemInterface::CallbackReturn SystemInterface::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
    {
    RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Started  driver");
    return CallbackReturn::SUCCESS;
    }

    hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
//   delto_client->disconnect();

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Stopped driver");
  return CallbackReturn::SUCCESS;
}

    void SystemInterface::init()
    {
        delto_client->connect();
    }


SystemInterface::return_type SystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  try {
    if (!delto_client) {
      std::cerr << "Client is not initialized" << std::endl;
      return return_type::ERROR;
    }

    auto received_data = delto_client->get_data();

    // 1. 위치 데이터 읽기
    std::vector<double> new_positions;
    try {

      if (received_data.joint.size() != 12)
      {
        return return_type::ERROR;
      }
      
      new_positions = received_data.joint;
    }
    catch(const std::exception& e ) {
      std::cerr << "Failed to read positions: " << e.what() << std::endl;
      return return_type::ERROR;
    }

    // 데이터 크기 검증
    if (new_positions.size() < positions.size()) {
      std::cerr << "Insufficient position data. Expected: " << positions.size() 
                << ", Got: " << new_positions.size() << std::endl;
      return return_type::ERROR;
    }

    // 2. 속도 계산
    double dt = period.seconds();
    for (size_t i = 0; i < positions.size(); ++i) {
      if (dt > 0 && !std::isnan(positions[i])) {
        velocities[i] = (new_positions[i] - positions[i]) / dt;
      } else {
        velocities[i] = 0.0;
      }
      
      positions[i] = new_positions[i];
    }

    return return_type::OK;
  }
  catch(const std::exception& e) {
    std::cerr << "Unexpected error in read: " << e.what() << std::endl;
    return return_type::ERROR;
  }
}

SystemInterface::return_type SystemInterface::write(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
 // JointControl을 통해 제어 입력 계산
  std::vector<double> u = JointControl(position_commands, positions, velocities, p_gain, d_gain);
  
  // calc_duty를 통해 duty값 계산
  std::vector<double> _duty = calc_duty(u);
  std::vector<int> int_duty(_duty.size());

  for (size_t i = 0; i < _duty.size(); ++i) {
    int_duty[i] = static_cast<int>(_duty[i]*10);
  }

  // 계산된 _duty를 send_duty로 전달 100.0 => 1000
  delto_client->send_duty(int_duty);  // calc_duty 대신 _duty를 사용

  return hardware_interface::return_type::OK;  // return 문 추가
}

std::vector<double> SystemInterface::JointControl(std::vector<double> target_joint_state,
                                                  std::vector<double> current_joint_state,
                                                  std::vector<double> joint_dot,
                                                  std::vector<double> kp,
                                                  std::vector<double> kd)
{

    std::vector<double> tq_u(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        tq_u[i] = kp[i] * (target_joint_state[i] - current_joint_state[i]) - (kd[i] * joint_dot[i]);
    }
    return tq_u;
}

std::vector<double> SystemInterface::calc_duty(std::vector<double> tq_u)
{

    std::vector<double> duty(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        double v = 14.1 / 0.8 * tq_u[i];

        duty[i] = 100.0 * v / 11.1;

        if (duty[i] > 100.0)
        {
            duty[i] = 100.0;
        }
        else if (duty[i] < -100.0)
        {
            duty[i] = -100.0;
        }
    }

    return duty;
}
} // namespace delto_3f

#include  "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(delto_3f_driver::SystemInterface, hardware_interface::SystemInterface)
