#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "delto_5f_TCP.hpp"

namespace delto_5f_driver
{
    class SystemInterface : public hardware_interface::SystemInterface
    {   
    public:
        using return_type = hardware_interface::return_type;
        RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterface)

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(
                    const std::vector<std::string> & start_interfaces,
                    const std::vector<std::string> & stop_interfaces) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        // CallbackReturn write() override;
        // CallbackReturn read() override;
    private:
        void init();
        std::unique_ptr<Delto5F_TCP> delto_client;

        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> efforts;

        std::vector<double> position_commands;
        std::thread m_init_thread;

        std::string delto_ip;
        uint16_t delto_port;

    };
}; // namespace delto_5f_driver
