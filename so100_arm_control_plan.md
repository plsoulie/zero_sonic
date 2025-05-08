# SO-100 Arm Control with Waveshare Servo Bus Drivers in ROS 2

## 1. Overview

This document outlines the implementation plan for controlling two SO-100 robotic arms using Waveshare servo bus drivers within the LeRobot framework, integrated into a ROS 2 architecture. The implementation follows the structure defined in the mobile manipulator architecture document.

## 2. Package Structure

```
arm_control/
├── include/
│   ├── arm_control/
│   │   ├── waveshare_hardware_interface.hpp
│   │   └── servo_utils.hpp
├── src/
│   ├── left_arm_controller_node.cpp
│   ├── right_arm_controller_node.cpp
│   ├── waveshare_hardware_interface.cpp
│   └── servo_utils.cpp
├── config/
│   ├── left_arm_config.yaml
│   ├── right_arm_config.yaml
│   └── controller_config.yaml
├── launch/
│   ├── arm_controllers.launch.py
│   └── single_arm.launch.py
├── urdf/
│   └── so100_arm.urdf.xacro
└── CMakeLists.txt
└── package.xml
```

## 3. Node Architecture

### 3.1 Controller Nodes

Each arm will have a dedicated controller node:

| Node                   | Namespace    | Purpose                                |
| ---------------------- | ------------ | -------------------------------------- |
| `left_arm_controller`  | `/left_arm`  | Controls the left SO-100 follower arm  |
| `right_arm_controller` | `/right_arm` | Controls the right SO-100 follower arm |

### 3.2 Hardware Interface

A custom hardware interface for the Waveshare servo bus drivers:

- Implements the `ros2_control` hardware interface classes
- Handles communication with the physical servo bus
- Translates between ROS commands and servo-specific protocols
- Integrates with the LeRobot framework

## 4. Communication Interfaces

### 4.1 Subscribed Topics

| Topic                               | Type                              | Purpose                               |
| ----------------------------------- | --------------------------------- | ------------------------------------- |
| `<arm_prefix>/command`              | `trajectory_msgs/JointTrajectory` | Joint trajectory commands for the arm |
| `<arm_prefix>/single_joint_command` | `std_msgs/Float64MultiArray`      | Direct command for individual joints  |
| `/teleop/arm_sync_active`           | `std_msgs/Bool`                   | Enable/disable teleop control         |

### 4.2 Published Topics

| Topic                           | Type                                          | Purpose                                          |
| ------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `<arm_prefix>/joint_states`     | `sensor_msgs/JointState`                      | Current joint positions, velocities, and efforts |
| `<arm_prefix>/controller_state` | `control_msgs/JointTrajectoryControllerState` | Controller state information                     |

### 4.3 Services

| Service                         | Type               | Purpose                        |
| ------------------------------- | ------------------ | ------------------------------ |
| `<arm_prefix>/calibrate`        | `std_srvs/Trigger` | Trigger arm calibration        |
| `<arm_prefix>/reset`            | `std_srvs/Trigger` | Reset the arm to home position |
| `<arm_prefix>/set_servo_torque` | Custom             | Enable/disable servo torque    |

## 5. Configuration Parameters

### 5.1 Hardware Configuration

```yaml
waveshare_servo_bus:
  bus_port: "/dev/ttyUSB0" # Adjust for each arm
  baud_rate: 1000000
  protocol: 2 # Depending on servo protocol
  update_rate: 50 # Hz
```

### 5.2 Joint Configuration

```yaml
joints:
  - name: "shoulder_pan_joint"
    id: 1
    min_position: -3.14
    max_position: 3.14
    max_velocity: 1.0
    max_effort: 1.0
  - name: "shoulder_lift_joint"
    id: 2
    min_position: -1.57
    max_position: 1.57
    max_velocity: 0.8
    max_effort: 1.0
  # ... additional joints
```

### 5.3 Controller Configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50 # Hz
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      # ... additional joints
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.01
```

## 6. Integration with Overall System

### 6.1 TF Frame Integration

Following the TF tree from the architecture document:

```
odom → base_link → lift_link → {left_arm_base, right_arm_base} → ... → wrist_link → camera_optical_frame
```

### 6.2 Teleop Integration

The arm controllers will:

- Accept commands from the `arm_mapper_node` in the `arm_teleop` package
- Scale and limit commands based on safety parameters
- Switch between teleop and autonomous control modes

### 6.3 Launch Integration

The arm controllers will be launched as part of the overall system:

```python
# Example launch snippet
arm_controller_left = Node(
    package='arm_control',
    executable='arm_controller_node',
    namespace='left_arm',
    parameters=[
        {'arm_prefix': 'left_arm'},
        {'bus_port': '/dev/ttyUSB0'},
        left_arm_config_path,
        controller_config_path
    ],
    remappings=[
        ('joint_states', 'joint_states'),
        ('command', 'command')
    ]
)
```

## 7. Safety Considerations

- Implement torque limiting to prevent arm damage
- Configure velocity and acceleration limits
- Add collision detection for self-collisions
- Implement command timeouts (stop if no new commands received)
- Add emergency stop handler

## 8. Future Extensions

- Integration with MoveIt 2 for motion planning
- Addition of force/impedance control
- Gripper integration
- Visual servoing capabilities

This document serves as a reference guide for implementing the SO-100 arm control system with Waveshare servo bus drivers in the ROS 2 architecture.
