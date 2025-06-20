#!/usr/bin/env python3

"""
Enhanced Human Worker Controller for Webots Pedestrian Proto

This controller manages realistic human worker behavior in the construction site
using the official Webots Pedestrian proto. It includes proper movement patterns,
work zones, and boundary awareness to prevent workers from leaving the site.
"""

from controller import Robot
import random
import math
import sys
import time

class HumanWorkerController:
    def __init__(self, worker_id="worker_1"):
        self.robot = Robot()
        self.worker_id = worker_id
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize sensors and actuators for Pedestrian proto
        self.init_devices()
        
        # Construction site boundaries (matching the arena)
        self.boundaries = {
            'x_min': -23.0, 'x_max': 23.0,
            'y_min': -23.0, 'y_max': 23.0
        }
        
        # Worker-specific configuration
        self.configure_worker()
        
        # Movement and behavior state
        self.current_state = "idle"
        self.state_timer = 0
        self.target_position = [0, 0]
        self.current_position = [0, 0]
        self.movement_speed = 0.5  # m/s walking speed
        
        # Work patterns
        self.work_schedule = self.create_work_schedule()
        self.current_task_index = 0
        
        print(f"Human Worker {self.worker_id} controller initialized")
        print(f"Work zone: {self.work_zone}")
    
    def init_devices(self):
        """Initialize sensors and actuators for Pedestrian proto"""
        try:
            # GPS for position tracking
            self.gps = self.robot.getDevice('gps')
            if self.gps:
                self.gps.enable(self.timestep)
            
            # Compass for orientation
            self.compass = self.robot.getDevice('compass')
            if self.compass:
                self.compass.enable(self.timestep)
            
            # For Pedestrian proto, we don't have direct motor control
            # Movement is controlled through the supervisor or physics
            
        except Exception as e:
            print(f"Warning: Could not initialize some devices: {e}")
            # Continue without devices if needed
            self.gps = None
            self.compass = None
    
    def configure_worker(self):
        """Configure worker-specific parameters based on worker ID"""
        if self.worker_id == "worker_1":
            self.work_zone = {'center': [10, 8], 'radius': 8}
            self.work_type = "scaffolding"
            self.break_frequency = 0.15  # 15% chance per minute
            
        elif self.worker_id == "worker_2":
            self.work_zone = {'center': [-8, 12], 'radius': 6}
            self.work_type = "materials"
            self.break_frequency = 0.12  # 12% chance per minute
            
        elif self.worker_id == "worker_3":
            self.work_zone = {'center': [5, -10], 'radius': 7}
            self.work_type = "equipment"
            self.break_frequency = 0.18  # 18% chance per minute
            
        else:
            # Default configuration
            self.work_zone = {'center': [0, 0], 'radius': 5}
            self.work_type = "general"
            self.break_frequency = 0.15
    
    def create_work_schedule(self):
        """Create a realistic work schedule for the worker"""
        schedule = []
        
        # Work tasks based on worker type
        if self.work_type == "scaffolding":
            tasks = [
                {"action": "work", "location": [12, 5], "duration": 180},  # 3 minutes
                {"action": "walk", "location": [15, 8], "duration": 30},   # 30 seconds
                {"action": "work", "location": [15, 8], "duration": 240},  # 4 minutes
                {"action": "break", "location": [10, 10], "duration": 120}, # 2 minutes
                {"action": "walk", "location": [13, 6], "duration": 20},
                {"action": "work", "location": [13, 6], "duration": 300},  # 5 minutes
            ]
        elif self.work_type == "materials":
            tasks = [
                {"action": "work", "location": [-15, -10], "duration": 150},
                {"action": "walk", "location": [-8, -15], "duration": 40},
                {"action": "work", "location": [-8, -15], "duration": 200},
                {"action": "break", "location": [-5, -12], "duration": 90},
                {"action": "walk", "location": [-12, -8], "duration": 35},
                {"action": "work", "location": [-12, -8], "duration": 220},
            ]
        else:  # equipment worker
            tasks = [
                {"action": "work", "location": [18, -5], "duration": 160},
                {"action": "walk", "location": [5, -15], "duration": 50},
                {"action": "work", "location": [5, -15], "duration": 180},
                {"action": "break", "location": [8, -8], "duration": 100},
                {"action": "walk", "location": [12, -12], "duration": 30},
                {"action": "work", "location": [12, -12], "duration": 250},
            ]
        
        return tasks
    
    def get_current_position(self):
        """Get current position from GPS or use stored position"""
        if self.gps:
            try:
                pos = self.gps.getValues()
                self.current_position = [pos[0], pos[1]]
                return self.current_position
            except:
                pass
        
        # Return stored position if GPS not available
        return self.current_position
    
    def get_current_orientation(self):
        """Get current orientation from compass"""
        if self.compass:
            try:
                north = self.compass.getValues()
                angle = math.atan2(north[1], north[0])
                return angle
            except:
                pass
        return 0.0
    
    def is_within_boundaries(self, position):
        """Check if position is within construction site boundaries"""
        x, y = position
        return (self.boundaries['x_min'] <= x <= self.boundaries['x_max'] and
                self.boundaries['y_min'] <= y <= self.boundaries['y_max'])
    
    def clamp_to_boundaries(self, position):
        """Clamp position to stay within boundaries"""
        x, y = position
        x = max(self.boundaries['x_min'], min(self.boundaries['x_max'], x))
        y = max(self.boundaries['y_min'], min(self.boundaries['y_max'], y))
        return [x, y]
    
    def distance_to_point(self, pos1, pos2):
        """Calculate distance between two points"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def generate_work_position(self):
        """Generate a random work position within the worker's zone"""
        center = self.work_zone['center']
        radius = self.work_zone['radius']
        
        # Generate random position within work zone
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, radius * 0.8)  # Stay within 80% of radius
        
        x = center[0] + distance * math.cos(angle)
        y = center[1] + distance * math.sin(angle)
        
        # Ensure position is within boundaries
        position = self.clamp_to_boundaries([x, y])
        return position
    
    def update_behavior_state(self):
        """Update the worker's behavior state based on schedule and conditions"""
        current_time = self.robot.getTime()
        
        # Get current task from schedule
        if self.current_task_index < len(self.work_schedule):
            current_task = self.work_schedule[self.current_task_index]
        else:
            # Restart schedule
            self.current_task_index = 0
            current_task = self.work_schedule[0]
        
        # Check if current state should change
        if self.state_timer <= 0:
            # Move to next task
            self.current_state = current_task["action"]
            self.state_timer = current_task["duration"]
            
            if "location" in current_task:
                target = current_task["location"]
                # Ensure target is within boundaries
                self.target_position = self.clamp_to_boundaries(target)
            else:
                self.target_position = self.generate_work_position()
            
            self.current_task_index = (self.current_task_index + 1) % len(self.work_schedule)
            
            print(f"{self.worker_id}: Starting {self.current_state} at {self.target_position}")
        
        # Decrease state timer
        self.state_timer -= self.timestep / 1000.0  # Convert to seconds
    
    def simulate_movement(self):
        """Simulate realistic human movement patterns"""
        current_pos = self.get_current_position()
        
        if self.current_state in ["walk", "work"]:
            # Calculate direction to target
            dx = self.target_position[0] - current_pos[0]
            dy = self.target_position[1] - current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0.5:  # If not close enough to target
                # Normalize direction and apply movement speed
                move_distance = self.movement_speed * (self.timestep / 1000.0)
                
                if distance > move_distance:
                    # Move towards target
                    ratio = move_distance / distance
                    new_x = current_pos[0] + dx * ratio
                    new_y = current_pos[1] + dy * ratio
                else:
                    # Reached target
                    new_x, new_y = self.target_position
                
                # Ensure new position is within boundaries
                new_position = self.clamp_to_boundaries([new_x, new_y])
                self.current_position = new_position
                
                # For Pedestrian proto, we would normally use the supervisor
                # to set the position, but for simulation we'll track it internally
                
        elif self.current_state == "idle":
            # Stay in place or make small random movements
            if random.random() < 0.1:  # 10% chance to make small movement
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_pos = [current_pos[0] + offset_x, current_pos[1] + offset_y]
                self.current_position = self.clamp_to_boundaries(new_pos)
    
    def handle_robot_interaction(self):
        """Handle interaction with the Go2 robot if nearby"""
        # This would typically involve checking distance to robot
        # and modifying behavior accordingly
        
        # For now, we'll implement a simple awareness system
        robot_position = [0, 0]  # Would get this from supervisor or communication
        current_pos = self.get_current_position()
        distance_to_robot = self.distance_to_point(current_pos, robot_position)
        
        if distance_to_robot < 3.0:  # Robot within 3 meters
            # Worker becomes aware of robot
            if random.random() < 0.3:  # 30% chance to look at robot
                print(f"{self.worker_id}: Noticed robot nearby")
                # Could modify behavior here (pause work, wave, etc.)
    
    def run(self):
        """Main control loop"""
        print(f"{self.worker_id}: Starting work simulation")
        
        step_count = 0
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            
            # Update behavior state
            self.update_behavior_state()
            
            # Simulate movement
            self.simulate_movement()
            
            # Handle robot interactions
            self.handle_robot_interaction()
            
            # Debug output every 5 seconds
            if step_count % (5000 // self.timestep) == 0:
                pos = self.get_current_position()
                print(f"{self.worker_id}: State={self.current_state}, "
                      f"Pos=[{pos[0]:.1f}, {pos[1]:.1f}], "
                      f"Target=[{self.target_position[0]:.1f}, {self.target_position[1]:.1f}]")

def main():
    """Main function to handle command line arguments and start controller"""
    worker_id = "worker_1"  # Default
    
    # Get worker ID from command line arguments
    if len(sys.argv) > 1:
        worker_id = sys.argv[1]
    
    # Create and run controller
    controller = HumanWorkerController(worker_id)
    controller.run()

if __name__ == "__main__":
    main() 