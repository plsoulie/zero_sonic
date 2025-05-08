#!/usr/bin/env python3

import sys
import math
import random
import time
import argparse
import pygame
from pygame.locals import *

class RobotEyes:
    def __init__(self, width=800, height=480, fullscreen=False, blink_interval=(3, 10)):
        """
        Initialize the robot eyes display
        
        Args:
            width: The width of the display (default 800 matches 7" RPi display)
            height: The height of the display (default 480 matches 7" RPi display)
            fullscreen: Whether to run in fullscreen mode
            blink_interval: Tuple of (min, max) seconds between blinks
        """
        self.width = width
        self.height = height
        self.fullscreen = fullscreen
        self.min_blink_interval, self.max_blink_interval = blink_interval
        
        # Eye properties
        self.eye_color = (0, 230, 0)  # Bright green
        self.eye_radius = int(height * 0.3)  # 30% of screen height
        self.eye_spacing = int(width * 0.2)  # 20% of screen width
        self.pupil_color = (0, 0, 0)  # Black
        self.pupil_radius = int(self.eye_radius * 0.3)  # 30% of eye radius
        
        # Eye positions
        self.left_eye_pos = (width // 2 - self.eye_spacing // 2, height // 2)
        self.right_eye_pos = (width // 2 + self.eye_spacing // 2, height // 2)
        
        # Pupil positions will be updated in the animation
        self.left_pupil_offset = [0, 0]
        self.right_pupil_offset = [0, 0]
        
        # Blinking state
        self.is_blinking = False
        self.blink_start_time = 0
        self.blink_duration = 0.15  # seconds
        self.next_blink_time = time.time() + self._get_random_blink_interval()
        
        # Movement parameters
        self.max_pupil_offset = self.eye_radius - self.pupil_radius
        self.pupil_movement_speed = 0.1  # Lower values = smoother movement
        self.target_left_pupil = [0, 0]
        self.target_right_pupil = [0, 0]
        self.next_movement_time = time.time() + random.uniform(1, 3)
        
        # Initialize Pygame
        pygame.init()
        pygame.mouse.set_visible(False)
        
        # Set up display
        self.display_flags = pygame.FULLSCREEN if fullscreen else 0
        self.screen = pygame.display.set_mode((width, height), self.display_flags)
        pygame.display.set_caption("Robot Eyes")
        
        # Create background
        self.background_color = (40, 40, 40)  # Dark grey
        
        # Clock for controlling frame rate
        self.clock = pygame.time.Clock()
        self.running = True
    
    def _get_random_blink_interval(self):
        """Generate a random time interval between blinks"""
        return random.uniform(self.min_blink_interval, self.max_blink_interval)
    
    def _interpolate(self, start, end, factor):
        """Linearly interpolate between two values"""
        return start + factor * (end - start)
    
    def _generate_new_targets(self):
        """Generate new random targets for pupils to look at"""
        # Random relative offsets within the eye
        max_offset = self.max_pupil_offset * 0.8  # Stay away from edges
        self.target_left_pupil = [
            random.uniform(-max_offset, max_offset),
            random.uniform(-max_offset, max_offset)
        ]
        
        # Usually both eyes look at the same point, but sometimes add variation
        if random.random() < 0.8:  # 80% chance eyes look at same thing
            self.target_right_pupil = self.target_left_pupil.copy()
        else:
            self.target_right_pupil = [
                random.uniform(-max_offset, max_offset),
                random.uniform(-max_offset, max_offset)
            ]
    
    def update(self):
        """Update eye state for animation"""
        current_time = time.time()
        
        # Check if it's time to blink
        if not self.is_blinking and current_time >= self.next_blink_time:
            self.is_blinking = True
            self.blink_start_time = current_time
            self.next_blink_time = current_time + self.blink_duration + self._get_random_blink_interval()
        
        # Check if blinking is complete
        if self.is_blinking and current_time >= self.blink_start_time + self.blink_duration:
            self.is_blinking = False
        
        # Check if it's time to move pupils
        if current_time >= self.next_movement_time:
            self._generate_new_targets()
            self.next_movement_time = current_time + random.uniform(1, 3)
        
        # Smoothly move pupils toward targets
        for i in range(2):
            self.left_pupil_offset[i] = self._interpolate(
                self.left_pupil_offset[i], 
                self.target_left_pupil[i], 
                self.pupil_movement_speed
            )
            self.right_pupil_offset[i] = self._interpolate(
                self.right_pupil_offset[i], 
                self.target_right_pupil[i], 
                self.pupil_movement_speed
            )
    
    def draw(self):
        """Draw the eyes on the screen"""
        # Fill background
        self.screen.fill(self.background_color)
        
        # Draw eyes (white circles)
        if not self.is_blinking:
            # Normal eyes
            
            # Draw left eye
            pygame.draw.circle(
                self.screen, 
                self.eye_color, 
                self.left_eye_pos, 
                self.eye_radius
            )
            
            # Draw right eye
            pygame.draw.circle(
                self.screen, 
                self.eye_color, 
                self.right_eye_pos, 
                self.eye_radius
            )
            
            # Draw pupils (black circles)
            left_pupil_pos = (
                int(self.left_eye_pos[0] + self.left_pupil_offset[0]),
                int(self.left_eye_pos[1] + self.left_pupil_offset[1])
            )
            right_pupil_pos = (
                int(self.right_eye_pos[0] + self.right_pupil_offset[0]),
                int(self.right_eye_pos[1] + self.right_pupil_offset[1])
            )
            
            pygame.draw.circle(
                self.screen, 
                self.pupil_color, 
                left_pupil_pos, 
                self.pupil_radius
            )
            pygame.draw.circle(
                self.screen, 
                self.pupil_color, 
                right_pupil_pos, 
                self.pupil_radius
            )
        else:
            # Blinking eyes - draw as lines
            blink_y = self.left_eye_pos[1]
            pygame.draw.line(
                self.screen,
                self.eye_color,
                (self.left_eye_pos[0] - self.eye_radius, blink_y),
                (self.left_eye_pos[0] + self.eye_radius, blink_y),
                width=5
            )
            pygame.draw.line(
                self.screen,
                self.eye_color,
                (self.right_eye_pos[0] - self.eye_radius, blink_y),
                (self.right_eye_pos[0] + self.eye_radius, blink_y),
                width=5
            )
        
        # Update display
        pygame.display.flip()
    
    def run(self):
        """Main loop to run the animation"""
        try:
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == QUIT:
                        self.running = False
                    elif event.type == KEYDOWN:
                        if event.key == K_ESCAPE or event.key == K_q:
                            self.running = False
                
                # Update and draw
                self.update()
                self.draw()
                
                # Cap the frame rate
                self.clock.tick(30)
        
        except KeyboardInterrupt:
            pass
        finally:
            # Clean up
            pygame.quit()
            print("Exited robot eyes display")


def main():
    """Parse command line arguments and run the eyes display"""
    parser = argparse.ArgumentParser(description="Display animated robot eyes")
    parser.add_argument("--width", type=int, default=800, 
                        help="Display width (default: 800)")
    parser.add_argument("--height", type=int, default=480, 
                        help="Display height (default: 480)")
    parser.add_argument("--fullscreen", action="store_true", 
                        help="Run in fullscreen mode")
    parser.add_argument("--color", type=str, default="green", 
                        help="Eye color (green, blue, red, yellow)")
    
    args = parser.parse_args()
    
    # Map color names to RGB values
    color_map = {
        "green": (0, 230, 0),
        "blue": (0, 180, 255),
        "red": (255, 30, 30),
        "yellow": (255, 230, 0),
        "purple": (180, 0, 255),
        "cyan": (0, 255, 255),
        "white": (255, 255, 255),
    }
    
    # Create and run the eyes
    eyes = RobotEyes(
        width=args.width,
        height=args.height,
        fullscreen=args.fullscreen
    )
    
    # Set eye color if specified
    if args.color.lower() in color_map:
        eyes.eye_color = color_map[args.color.lower()]
    
    eyes.run()


if __name__ == "__main__":
    main() 