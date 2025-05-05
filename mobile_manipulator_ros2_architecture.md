
# Modular ROS&nbsp;2 Architecture for a Teleoperated Mobile Manipulator

## 1. System Overview
This mobile manipulator features:
- **Base:** Differential‑drive chassis (4 × DC motors) plus a vertical shoulder‑lift (stepper).
- **Manipulators:** Two SO‑100 follower arms mounted on the shoulder‑lift, and two SO‑100 leader arms used as master devices.
- **I/O:** Xbox 360 joystick for driving, dual arm‑mounted cameras for operator vision, and a 7″ on‑board display for status/GUI.
- **Compute:** NVIDIA Jetson Orin running ROS 2 Humble.

The architecture is _teleoperation‑first_ but **modular** so higher‑level autonomy stacks (Nav2, MoveIt 2, perception) can be dropped in later with minimal refactor.

## 2. Package & Node Organization
| Package | Primary Nodes | Purpose |
|---------|---------------|---------|
| **base_control** | `drive_controller`, `shoulder_lift_controller`, `hardware_interface` | Convert `/cmd_vel` + lift commands to H‑bridge & stepper signals; publish odom & `/joint_states`. |
| **arm_control** | `left_arm_controller`, `right_arm_controller` | ros2_control interfaces for follower arms via Waveshare bus‑servo adapter. |
| **leader_arms** | `leader_left_node`, `leader_right_node` | Publish leader‑arm joint states for teleop mapping. |
| **arm_teleop** | `arm_mapper_node` | Map leader joint angles → follower arm commands; safety & scaling. |
| **joy_teleop** | `joy_node`, `teleop_twist_node` | Xbox 360 joystick → `/cmd_vel` (and lift) commands. |
| **twist_mux** | `twist_mux` | Prioritise `/teleop/cmd_vel` vs `/nav/cmd_vel` for the base. |
| **vision** | `left_cam_node`, `right_cam_node` | Publish raw/compressed images; future CV nodes subscribe here. |
| **ui_dashboard** | `ui_node` | Touch GUI on 7″ display showing status, video, mode switch, e‑stop. |
| **supervisor** | `mode_manager` (optional) | Lifecycle or service‑based enable/disable of teleop vs autonomy. |

_All nodes live in separate namespaces (`/left_arm`, `/right_arm`, etc.) and communicate exclusively via ROS interfaces for loose coupling._

## 3. DDS Middleware Recommendation
- **Default:** **Fast DDS** for lowest latency & highest throughput on Jetson Orin (good for image + joint‑state traffic).  
- **Alternative:** **Cyclone DDS** if multi‑machine networking or long‑run stability issues arise.  
Switch by exporting `RMW_IMPLEMENTATION` at launch.

## 4. Core Topics & Services

### 4.1 Mobile Base
| Topic / Service | Type | Direction | Note |
|-----------------|------|-----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | **Sub** | Velocity commands (joy or Nav2). |
| `/odom` | `nav_msgs/Odometry` | **Pub** | Integrated wheel encoders. |
| `/joint_states` | `sensor_msgs/JointState` | **Pub** | Wheels + lift joint. |
| `/shoulder_lift/command` | `std_msgs/Float64` | **Sub** | Target lift position (or use joint traj). |

### 4.2 Manipulator Arms  
(*prefix every topic with `left_arm/` or `right_arm/`*)

| Topic / Action | Type | Direction |
|----------------|------|-----------|
| `command` | custom or `JointTrajectory` | **Sub** |
| `joint_states` | `sensor_msgs/JointState` | **Pub** |
| Leader arm states | `/leader/left_arm/joint_states` | **Pub** |
| Teleop enable | `/teleop/arm_sync_active` (`Bool`) | **Pub** |

### 4.3 Vision
`/left_arm_camera/image_raw`, `/left_arm_camera/camera_info`, … (right arm equivalent). Optional compressed transport variants.

### 4.4 Teleop & UI
- `/joy` → raw joystick.
- `/ui/emergency_stop` (`Bool`), `/mode` (`String/Enum`) for mode switching.
- Services: `/switch_mode`, `/reset_odom`, `/set_lift_height`, `/shutdown`.

## 5. TF Frame Tree
```
odom → base_link → lift_link → {left_arm_base, right_arm_base}
                                     ↘ wrist_link ↘ camera_optical_frame
```
Leader‑arm frames (if published) sit in an independent tree. Static transforms defined in URDF; dynamic ones from `robot_state_publisher` & diff‑drive controller.

## 6. Modularity for Autonomy
1. **Decoupled control interfaces**: Hardware nodes only consume `/cmd_vel` or `arm/command`.  
2. **Standard ROS 2 actions**: Arms expose `FollowJointTrajectory`, so MoveIt 2 can drop in.  
3. **Twist multiplexer**: Human input overrides autonomy safely.  
4. **Lifecycle or launch‑based mode manager** toggles teleop nodes on/off.  
5. **Scalable namespaces**: Adding a 3rd arm or sensors simply instantiates another node set.

## 7. Launch & Configuration Highlights
- **`bringup.launch.py`**: Starts base, arms, vision, UI, teleop. Key args:  
  - `rmw_implementation` (`fastdds`/`cyclonedds`)  
  - `joy_config.yaml`, `twist_mux.yaml`  
  - `arm_prefix:=left_arm`, etc.  
- **Parameter YAMLs**: wheel radius, PID gains, servo IDs, joint limits.  
- **DDS XML** (optional) for custom QoS profiles.  
- **Safety**: command timeouts, e‑stop listener, maximum velocities.

This markdown file captures the complete, modular ROS 2 architecture ready for initial teleoperation and seamless evolution toward semi‑ or fully‑autonomous operation.
