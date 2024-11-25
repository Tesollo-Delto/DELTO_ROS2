#pragma once

#include <map>
#include <memory>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace delto3F
{
constexpr char HW_IF_CURRENT[] = "current";

class SystemInterface : public hardware_interface::SystemInterface
{
public:
  using return_type = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void init();

  std::atomic<bool> m_initialized{false};
  std::thread m_init_thread;
  std::shared_ptr<rclcpp::Node> m_node;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_svh_initialized_publisher;

  std::vector<double> m_position_commands;

  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;
  std::vector<double> m_currents;

};
}  // namespace delto_3f  
