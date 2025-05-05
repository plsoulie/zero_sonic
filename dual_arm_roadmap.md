
# Development Roadmap – Dual‑Arm Teleoperated Mobile Manipulator

This roadmap compresses the critical path to **full bimanual teleoperation** while preserving a clean upgrade path toward autonomy. Each box represents ~2 weeks unless otherwise noted.

---

## 🏁 Sprint 0 – Project Foundations (≈ 1 week)

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| Reproducible dev & CI | • Git repo with `ros2_ws/` skeleton<br>• Dev‑container / Dockerfile for Jetson Orin & x86<br>• GitHub Actions running `colcon test` and linters | Clone → `colcon build` passes on laptop & Orin; CI green. |

---

## 🚗 Sprint 1 – Mobile Base + Lift MVP

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| Drive & lift platform from joystick | **base_control** pkg:<br>• H‑bridge hardware interface<br>• `diff_drive_controller` YAML<br>• Stepper lift node & URDF (`odom → base_link → lift_link`)<br>• Xbox 360 `teleop_twist_joy` mapping | Roll 5 m straight & rotate in place; bumpers raise/​lower lift; RViz shows odom. |

---

## 🦾 Sprint 2 – BOTH Follower Arms Online

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| Hardware control of left **and** right SO‑100 arms | Two `arm_controller` instances (`left_arm`, `right_arm`) via Waveshare<br>Joint‑group‑position controllers @ 50 Hz<br>URDF links for both arms | Publishing target angles moves each joint within ±1 °; TF reflects motion. |

---

## 🎮 Sprint 3 – Dual Leader‑Follower Teleop Loop

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| Real‑time bimanual mapping | • `leader_left` & `leader_right` drivers<br>• `arm_mapper_dual` node for mirroring + safety<br>• `/teleop/arm_sync_active` enable switch | Followers track leaders ≤ 200 ms latency while base drives; disable switch halts mapping. |

---

## 👀 Sprint 4 – Vision & UI Integration

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| Situational awareness for bimanual teleop | **vision** package (camera drivers + compressed topics)<br>**ui_dashboard** on 7″ display with dual video, joint bars, odom, e‑stop| Live dual video ≤ 200 ms; UI e‑stop halts base & arms < 50 ms. |

---

## 🔀 Sprint 5 – System Hardening & Soak Test

| Goal | Deliverables | Acceptance Criteria |
|------|--------------|---------------------|
| 2‑hour endurance with dual‑arm teleop | • `twist_mux` priority YAML<br>• `mode_manager` Lifecycle node for teleop ↔ autonomy stubs<br>• Logging + rosbag scripts for soak | Two‑hour session: base drives figure‑eights while operator stacks blocks bimanually—no node crashes, ≤ 1 % message loss. |

---

## ✨ Stretch Goals (Post‑Roadmap)

| Stretch Item | Value Add |
|--------------|-----------|
| **Nav2 + SLAM** | Hands‑free navigation while operator focuses on arms. |
| **MoveIt 2 Bimanual Planning** | Semi‑autonomous grasp sequences; teleop becomes supervisory. |
| **Perception Modules** | Object detection → pick‑point publishing for MoveIt. |
| **Remote Teleop (ROS 2 Web bridge)** | Operate robot from anywhere – powerful for demos & support. |

---

### Tracking & Process Tips

| Artifact | Purpose |
|----------|---------|
| **Kanban (GitHub Projects)** | Visual sprint backlog & WIP limits. |
| **Milestone tags** (`v0.1‑base`, `v0.2‑bimanual`, …) | Stable restore points. |
| **Weekly demo videos** | Early stakeholder feedback & morale boost. |
| **rosbag + pytest smoke tests** | Automated regression as nodes scale. |

Stay demo‑driven—each sprint should deliver something _visibly cooler_ than the last. By Sprint 3 you’ll be stacking blocks with two arms; by Sprint 5 your teleop stack will be robust enough to host autonomy experiments. Onward and upward 🚀
