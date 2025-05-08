#ifndef ARM_CONTROL_SERVO_UTILS_HPP_
#define ARM_CONTROL_SERVO_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace arm_control
{

// Abstract interface for servo communication
class ServoInterface
{
public:
  virtual ~ServoInterface() = default;

  // Connect to the servo bus
  virtual bool connect(const std::string & port, int baudrate) = 0;
  
  // Disconnect from the servo bus
  virtual void disconnect() = 0;
  
  // Check if a specific servo exists at the given ID
  virtual bool ping(int id) = 0;
  
  // Find all servo IDs on the bus
  virtual std::vector<int> scan_ids(int max_id = 252) = 0;
  
  // Read position from a servo
  virtual double read_position(int id) = 0;
  
  // Read velocity from a servo
  virtual double read_velocity(int id) = 0;
  
  // Read current/load from a servo (effort)
  virtual double read_effort(int id) = 0;
  
  // Write position to a servo
  virtual bool write_position(int id, double position) = 0;
  
  // Enable torque on a servo
  virtual bool enable_torque(int id, bool enable) = 0;
};

// Implementation of ServoInterface for the Waveshare servo bus using LeRobot's feetech.py
class WaveshareServoInterface : public ServoInterface
{
public:
  WaveshareServoInterface();
  ~WaveshareServoInterface();

  bool connect(const std::string & port, int baudrate) override;
  void disconnect() override;
  bool ping(int id) override;
  std::vector<int> scan_ids(int max_id = 252) override;
  double read_position(int id) override;
  double read_velocity(int id) override;
  double read_effort(int id) override;
  bool write_position(int id, double position) override;
  bool enable_torque(int id, bool enable) override;

private:
  // Pointer to the FeetechMotorsBus object from LeRobot
  // We use void* to avoid including feetech.py directly in the header
  void* motors_bus_;
  
  // Flag to track connection status
  bool connected_;
  
  // Model type for all servos (defaults to "sts3215")
  std::string servo_model_;
  
  // Map motor IDs to their associated model
  std::unordered_map<int, std::string> motor_models_;
};

// Factory function to create appropriate servo interface
std::unique_ptr<ServoInterface> create_servo_interface(const std::string & type = "waveshare");

}  // namespace arm_control

#endif  // ARM_CONTROL_SERVO_UTILS_HPP_
