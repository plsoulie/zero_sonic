#!/usr/bin/env python3

import argparse
import math
import random
import time
import numpy as np

# These imports would be from the LeRobot framework
try:
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
    from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
except ImportError:
    print("Warning: LeRobot framework not found. Running in simulation mode.")
    # Create mock classes for testing without the framework
    class FeetechMotorsBus:
        def __init__(self, config):
            self.config = config
            self.connected = False
            self.motor_positions = {name: 0.0 for name in config.motors.keys()}
            print(f"Mock FeetechMotorsBus initialized with motors: {list(config.motors.keys())}")
        
        def connect(self):
            self.connected = True
            print(f"Connected to port: {self.config.port}")
            return True
            
        def read(self, data_name, motor_names=None):
            if data_name == "Present_Position":
                return np.array([self.motor_positions[name] for name in motor_names])
            return np.zeros(len(motor_names))
            
        def write(self, data_name, values, motor_names=None):
            if data_name == "Goal_Position":
                for i, name in enumerate(motor_names):
                    self.motor_positions[name] = values[i]
                print(f"Writing positions for {motor_names}: {values}")
            return True

        def disconnect(self):
            self.connected = False
            print("Disconnected")

    class FeetechMotorsBusConfig:
        def __init__(self, port, motors, mock=True):
            self.port = port
            self.motors = motors
            self.mock = mock


class ArmRandomMotion:
    def __init__(self, left_port, right_port, amplitude=0.1, frequency=0.5):
        """
        Initialize the random motion generator for both arms.
        
        Args:
            left_port: Serial port for the left arm
            right_port: Serial port for the right arm
            amplitude: Maximum angle change in radians
            frequency: Approximate frequency of movement changes in Hz
        """
        self.amplitude = amplitude
        self.frequency = frequency
        self.left_arm = None
        self.right_arm = None
        self.left_port = left_port
        self.right_port = right_port
        
        # Keep track of current positions
        self.left_positions = None
        self.right_positions = None
        
        # Target positions
        self.left_targets = None
        self.right_targets = None
        
        # Time tracking
        self.last_target_update = 0
        self.target_duration = 1.0 / frequency
        
        # Motor configurations
        self.motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                           "wrist_flex", "wrist_roll", "gripper"]
        
        # Initialize the buses
        self._init_motors()
        
    def _init_motors(self):
        """Initialize motor buses for both arms"""
        # Left arm configuration
        left_config = FeetechMotorsBusConfig(
            port=self.left_port,
            motors={
                # name: (index, model)
                "shoulder_pan": [1, "sts3215"],
                "shoulder_lift": [2, "sts3215"],
                "elbow_flex": [3, "sts3215"],
                "wrist_flex": [4, "sts3215"],
                "wrist_roll": [5, "sts3215"],
                "gripper": [6, "sts3215"],
            }
        )
        
        # Right arm configuration
        right_config = FeetechMotorsBusConfig(
            port=self.right_port,
            motors={
                # name: (index, model)
                "shoulder_pan": [1, "sts3215"],
                "shoulder_lift": [2, "sts3215"],
                "elbow_flex": [3, "sts3215"],
                "wrist_flex": [4, "sts3215"],
                "wrist_roll": [5, "sts3215"],
                "gripper": [6, "sts3215"],
            }
        )
        
        # Create motor buses
        self.left_arm = FeetechMotorsBus(left_config)
        self.right_arm = FeetechMotorsBus(right_config)
    
    def connect(self):
        """Connect to both arms"""
        print("Connecting to left arm...")
        left_connected = self.left_arm.connect()
        
        print("Connecting to right arm...")
        right_connected = self.right_arm.connect()
        
        if left_connected and right_connected:
            print("Successfully connected to both arms")
            
            # Read initial positions
            self.left_positions = self.left_arm.read("Present_Position", self.motor_names)
            self.right_positions = self.right_arm.read("Present_Position", self.motor_names)
            
            # Set initial targets to current positions
            self.left_targets = np.copy(self.left_positions)
            self.right_targets = np.copy(self.right_positions)
            
            return True
        else:
            print("Failed to connect to arms")
            return False
    
    def generate_new_targets(self):
        """Generate new random target positions within amplitude limits of current position"""
        # Random offsets within amplitude
        left_offsets = np.random.uniform(-self.amplitude, self.amplitude, len(self.motor_names))
        right_offsets = np.random.uniform(-self.amplitude, self.amplitude, len(self.motor_names))
        
        # Set new targets
        self.left_targets = self.left_positions + left_offsets
        self.right_targets = self.right_positions + right_offsets
        
        print(f"New left targets: {self.left_targets}")
        print(f"New right targets: {self.right_targets}")
        
        # Update timestamp
        self.last_target_update = time.time()
    
    def interpolate_positions(self, start, end, progress):
        """Interpolate between start and end positions with a smoothing function"""
        # Smooth the progress using sine-based easing function
        smoothed = 0.5 - 0.5 * math.cos(progress * math.pi)
        return start + (end - start) * smoothed
    
    def update(self):
        """Update arm positions based on current targets and elapsed time"""
        current_time = time.time()
        elapsed = current_time - self.last_target_update
        
        # Generate new targets if needed
        if elapsed >= self.target_duration:
            # First move to the current targets
            self.left_positions = np.copy(self.left_targets)
            self.right_positions = np.copy(self.right_targets)
            
            # Then generate new targets
            self.generate_new_targets()
            elapsed = 0  # Reset elapsed time
        
        # Calculate progress ratio (0 to 1)
        progress = min(1.0, elapsed / self.target_duration)
        
        # Interpolate current positions
        left_interpolated = self.interpolate_positions(
            self.left_positions, self.left_targets, progress)
        right_interpolated = self.interpolate_positions(
            self.right_positions, self.right_targets, progress)
        
        # Write interpolated positions to motors
        self.left_arm.write("Goal_Position", left_interpolated, self.motor_names)
        self.right_arm.write("Goal_Position", right_interpolated, self.motor_names)
    
    def run(self, duration=60):
        """Run the random motion for a specified duration (in seconds)"""
        if not self.connect():
            print("Failed to connect. Exiting.")
            return
        
        print(f"Running random motion for {duration} seconds...")
        start_time = time.time()
        
        try:
            # Generate initial targets
            self.generate_new_targets()
            
            # Main loop
            while time.time() - start_time < duration:
                self.update()
                # 20Hz update rate
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("Motion stopped by user")
        finally:
            # Disconnect from motor buses
            self.left_arm.disconnect()
            self.right_arm.disconnect()
            print("Disconnected from both arms")


def main():
    """Main function to parse arguments and run the motion"""
    parser = argparse.ArgumentParser(description="Generate random smooth motion for SO-100 arms")
    parser.add_argument("--left-port", type=str, default="/dev/ttyUSB0", 
                        help="Serial port for left arm")
    parser.add_argument("--right-port", type=str, default="/dev/ttyUSB1", 
                        help="Serial port for right arm")
    parser.add_argument("--amplitude", type=float, default=0.1,
                        help="Maximum movement amplitude in radians")
    parser.add_argument("--frequency", type=float, default=0.5,
                        help="Frequency of motion changes in Hz")
    parser.add_argument("--duration", type=int, default=60,
                        help="Duration to run in seconds")
    
    args = parser.parse_args()
    
    # Create and run the motion controller
    controller = ArmRandomMotion(
        left_port=args.left_port,
        right_port=args.right_port,
        amplitude=args.amplitude,
        frequency=args.frequency
    )
    
    controller.run(duration=args.duration)


if __name__ == "__main__":
    main() 