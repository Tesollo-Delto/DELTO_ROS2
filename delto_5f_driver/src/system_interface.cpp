#include "delto_5f_driver/system_interface.hpp"

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

namespace delto_5f_driver
{
    
    
    hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::CallbackReturn::SUCCESS 
        != hardware_interface::SystemInterface::on_init(info))
        {
            return CallbackReturn::ERROR;
        }

        // positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // efforts.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
            positions.resize(info_.joints.size(), 0.0);
    velocities.resize(info_.joints.size(), 0.0);
    efforts.resize(info_.joints.size(), 0.0);
    
        position_commands.resize(info_.joints.size(), 0.0);

        // device_file = info.hardware_parameters["device_file"];
        delto_ip = info.hardware_parameters.at("delto_ip");
        delto_port = std::stoi(info.hardware_parameters.at("delto_port"));
        // m_client = std::make_unique<delto_5F_TCP>(device_file);
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
          joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[2].name == hardware_interface::HW_IF_EFFORT )) {
            RCLCPP_ERROR(
            rclcpp::get_logger("SystemInterface"),
            "Joint '%s' needs the following state interfaces in this order: %s, %s, %s, and %s.",
            joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT);
            return CallbackReturn::ERROR;
            }
    }
  
        delto_client = std::make_unique<Delto5F_TCP>(delto_ip, delto_port);
        m_init_thread = std::thread(&SystemInterface::init, this);
        m_init_thread.detach();


        return CallbackReturn::SUCCESS;
    }
hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
//   if (delto_client) {
//     // 필요한 정리 작업 수행
//     delto_client->disconnect();
//   }
  
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Deactivated driver");
  return CallbackReturn::SUCCESS;
}
  //   std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces()
  //   {
  //   std::vector<hardware_interface::StateInterface> state_interfaces;
  //   for (std::size_t i = 0; i < info_.joints.size(); i++) {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions[i]));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities[i]));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts[i]));
  // }

  // return state_interfaces;
  //   }

    // std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces()
    // {
    // std::vector<hardware_interface::CommandInterface> command_interfaces;
    // for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands[i]));
    // }

    std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 3);  // position, velocity, effort

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts[i]));
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
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Failed to connect to the device.");
        //     return;
        // }
    }

//     SystemInterface::return_type SystemInterface::read(
//   [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
// {
//     std::vector<double> current_position = delto_client->get_position_rad();
    
//     for (size_t i = 0; i < positions.size(); ++i) {
//     velocities[i] = current_position[i]-positions[i]/time.seconds(); //rad/s
//     }
//     positions = current_position;

//     std::vector<double> currents = delto_client->get_current();

//     for (size_t i = 0; i < currents.size(); ++i) {
//     efforts[i] = 375 * currents[i]; //mA to Nm
//     }
//   return return_type::OK;
// }

SystemInterface::return_type SystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  try {
    if (!delto_client) {
      std::cerr << "Client is not initialized" << std::endl;
      return return_type::ERROR;
    }

    // 1. 위치 데이터 읽기
    std::vector<double> new_positions;
    try {
      new_positions = delto_client->get_position_rad();
    }
    catch(const std::exception& e) {
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

    // 3. 전류 데이터 읽기
    std::vector<double> raw_currents(12);
    try {
      auto raw_currents = delto_client->get_current();
    }
    catch(const std::exception& e) {
      std::cerr << "Failed to read currents: " << e.what() << std::endl;
      return return_type::ERROR;
    }

    // 데이터 크기 검증
    if (raw_currents.size() < 12) {
      std::cerr << "Insufficient current electric data. Expected: " << efforts.size() 
                << ", Got: " << raw_currents.size() << std::endl;
      return return_type::ERROR;
    }

    // 4. 전류 데이터 변환
    for (size_t i = 0; i < efforts.size(); ++i) {
      // 원시 데이터를 토크로 변환 (여기서는 단순히 스케일링만 적용)
      efforts[i] = static_cast<double>(raw_currents[i]) * 0.001;  // mA to A
    }

    // // 디버그 출력
    // std::cout << "Current state:" << std::endl;
    // for (size_t i = 0; i < positions.size(); ++i) {
    //   std::cout << "Joint " << i << ":"
    //             << " pos=" << positions[i] 
    //             << " vel=" << velocities[i]
    //             << " eff=" << efforts[i] << std::endl;
    // }

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
  delto_client->set_postion_rad(position_commands);
  return return_type::OK;
}

} // namespace d

#include  "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(delto_5f_driver::SystemInterface, hardware_interface::SystemInterface)
