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

#include "delto_external_TCP.hpp"

namespace delto_3f_driver
{

class SystemInterface : public hardware_interface::SystemInterface
{
public:
  using return_type = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  std::vector<double> calc_duty(std::vector<double> tq_u);
  std::vector<double> JointControl(std::vector<double> target_joint_state,
                                   std::vector<double> current_joint_state,
                                   std::vector<double> joint_dot,
                                   std::vector<double> kp,
                                   std::vector<double> kd);

private:
  void init();

  std::atomic<bool> m_initialized{false};
  std::thread m_init_thread;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<DeltoTCP::Communication> delto_client;

  std::vector<double> position_commands;
  std::vector<double> positions;
  std::vector<double> velocities;
  
  std::string delto_ip;
  uint16_t delto_port;
  std::vector<double> p_gain;
  std::vector<double> d_gain;

};
}  // namespace delto_3f  
