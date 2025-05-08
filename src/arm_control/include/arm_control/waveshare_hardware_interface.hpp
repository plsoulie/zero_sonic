#ifndef ARM_CONTROL_WAVESHARE_HARDWARE_INTERFACE_HPP_
#define ARM_CONTROL_WAVESHARE_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_control/servo_utils.hpp"

namespace arm_control
{

class WaveshareHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WaveshareHardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the Waveshare servo bus driver
  std::string port_;
  int baudrate_;
  std::unordered_map<std::string, std::pair<int, std::string>> motors_;
  
  // The hardware interface to the servo bus
  std::unique_ptr<ServoInterface> servo_interface_;
  
  // Joint states (position, velocity, effort)
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  
  // Joint commands
  std::vector<double> hw_position_commands_;
  
  // Joint names
  std::vector<std::string> joint_names_;
};

}  // namespace arm_control

#endif  // ARM_CONTROL_WAVESHARE_HARDWARE_INTERFACE_HPP_
