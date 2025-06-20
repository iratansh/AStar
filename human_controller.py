#!/usr/bin/env python3
"""
Human Worker Controller for Webots Construction Site Simulation
Implements realistic movement patterns for construction workers
"""

import sys
import math
import random
import time
from controller import Robot, Motor, GPS

class HumanWorker:
    """
    Controller for human worker robots in the construction site
    Implements realistic movement patterns and behaviors
    """
    
    def __init__(self, worker_id):
        self.robot = Robot()
        self.worker_id = worker_id
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize sensors
        self.gps = self.robot.getDevice(f"{worker_id}_gps")
        if self.gps:
            self.gps.enable(self.timestep)
        
        # Movement parameters
        self.max_speed = 1.2  # m/s walking speed
        self.current_speed = 0.0
        self.target_position = None
        self.current_position = [0, 0, 0]
        
        # Behavior state
        self.state = "working"  # working, walking, break, idle
        self.state_start_time = 0
        self.work_duration = random.uniform(30, 120)  # 30-120 seconds
        self.break_duration = random.uniform(10, 30)   # 10-30 seconds
        self.walk_duration = random.uniform(5, 15)     # 5-15 seconds
        
        # Work zones and patterns based on worker ID
        self.work_zones = self._define_work_zones()
        self.current_zone_index = 0
        self.zone_visit_order = list(range(len(self.work_zones)))
        random.shuffle(self.zone_visit_order)
        
        # Movement tracking
        self.last_position = [0, 0, 0]
        self.movement_history = []
        self.idle_time = 0
        
        print(f"Human worker {worker_id} initialized")
    
    def _define_work_zones(self):
        """Define work zones based on worker ID"""
        if self.worker_id == "worker1":
            # Worker 1 focuses on scaffolding and material handling
            return [
                [-10, 5, 0],    # Scaffolding area
                [-15, -10, 0],  # Material storage
                [-8, -15, 0],   # Container area
                [-5, 0, 0],     # Central work area
            ]
        elif self.worker_id == "worker2":
            # Worker 2 works on equipment and pipe installation
            return [
                [10, 8, 0],     # Pipe installation area
                [12, -8, 0],    # Near excavator
                [18, 12, 0],    # Tool shed
                [5, 5, 0],      # Central coordination point
            ]
        elif self.worker_id == "worker3":
            # Worker 3 handles barriers and safety equipment
            return [
                [15, 0, 0],     # Concrete barriers
                [5, 3, 0],      # Safety cone area
                [7, -5, 0],     # Another safety area
                [0, 10, 0],     # Perimeter check
            ]
        else:
            # Default pattern for any additional workers
            return [
                [0, 0, 0],
                [5, 5, 0],
                [-5, -5, 0],
                [0, 8, 0],
            ]
    
    def get_position(self):
        """Get current GPS position"""
        if self.gps:
            pos = self.gps.getValues()
            self.current_position = [pos[0], pos[1], pos[2]]
            return self.current_position
        return self.current_position
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def move_towards_target(self, target, speed_factor=1.0):
        """
        Move towards target position
        Returns True if target reached, False otherwise
        """
        current_pos = self.get_position()
        
        # Calculate direction to target
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if target reached
        if distance < 0.5:  # 0.5m tolerance
            return True
        
        # Calculate movement direction
        if distance > 0:
            move_x = (dx / distance) * self.max_speed * speed_factor
            move_y = (dy / distance) * self.max_speed * speed_factor
            
            # Update position (simplified movement for simulation)
            new_x = current_pos[0] + move_x * (self.timestep / 1000.0)
            new_y = current_pos[1] + move_y * (self.timestep / 1000.0)
            
            # Store movement history
            self.movement_history.append({
                'time': self.robot.getTime(),
                'position': [new_x, new_y, current_pos[2]],
                'target': target,
                'state': self.state
            })
            
            # Limit history size
            if len(self.movement_history) > 1000:
                self.movement_history = self.movement_history[-500:]
        
        return False
    
    def select_next_work_zone(self):
        """Select next work zone to visit"""
        # Move to next zone in shuffled order
        self.current_zone_index = (self.current_zone_index + 1) % len(self.zone_visit_order)
        zone_idx = self.zone_visit_order[self.current_zone_index]
        
        # Add some randomness to the exact position within the zone
        base_pos = self.work_zones[zone_idx]
        random_offset_x = random.uniform(-2, 2)
        random_offset_y = random.uniform(-2, 2)
        
        return [
            base_pos[0] + random_offset_x,
            base_pos[1] + random_offset_y,
            base_pos[2]
        ]
    
    def update_behavior_state(self):
        """Update worker behavior state based on time and conditions"""
        current_time = self.robot.getTime()
        time_in_state = current_time - self.state_start_time
        
        if self.state == "working":
            if time_in_state > self.work_duration:
                # Decide next action: break or move to new work zone
                if random.random() < 0.3:  # 30% chance of break
                    self.state = "break"
                    self.break_duration = random.uniform(10, 30)
                else:
                    self.state = "walking"
                    self.target_position = self.select_next_work_zone()
                    self.walk_duration = random.uniform(5, 15)
                
                self.state_start_time = current_time
                
        elif self.state == "walking":
            # Check if target reached or time exceeded
            if (self.target_position and 
                self.move_towards_target(self.target_position, speed_factor=0.8)):
                self.state = "working"
                self.work_duration = random.uniform(30, 120)
                self.state_start_time = current_time
                self.target_position = None
            elif time_in_state > self.walk_duration:
                # Timeout - switch to working at current location
                self.state = "working"
                self.work_duration = random.uniform(30, 120)
                self.state_start_time = current_time
                self.target_position = None
                
        elif self.state == "break":
            if time_in_state > self.break_duration:
                # End break - either start working or move to new location
                if random.random() < 0.7:  # 70% chance to work at current location
                    self.state = "working"
                    self.work_duration = random.uniform(30, 120)
                else:
                    self.state = "walking"
                    self.target_position = self.select_next_work_zone()
                    self.walk_duration = random.uniform(5, 15)
                
                self.state_start_time = current_time
        
        elif self.state == "idle":
            if time_in_state > 5:  # End idle state after 5 seconds
                self.state = "walking"
                self.target_position = self.select_next_work_zone()
                self.state_start_time = current_time
    
    def simulate_work_activity(self):
        """Simulate small movements during work to appear active"""
        if self.state == "working":
            current_pos = self.get_position()
            
            # Small random movements around work position
            small_movement_x = random.uniform(-0.1, 0.1)
            small_movement_y = random.uniform(-0.1, 0.1)
            
            # Occasionally make larger adjustments
            if random.random() < 0.1:  # 10% chance
                small_movement_x *= 3
                small_movement_y *= 3
    
    def get_worker_status(self):
        """Get current worker status for external monitoring"""
        return {
            'worker_id': self.worker_id,
            'position': self.get_position(),
            'state': self.state,
            'target': self.target_position,
            'time_in_state': self.robot.getTime() - self.state_start_time,
            'current_zone': self.current_zone_index
        }
    
    def run(self):
        """Main control loop"""
        print(f"Starting human worker {self.worker_id} control loop")
        
        # Initialize state
        self.state_start_time = self.robot.getTime()
        self.target_position = self.select_next_work_zone()
        
        while self.robot.step(self.timestep) != -1:
            # Update behavior state
            self.update_behavior_state()
            
            # Perform actions based on current state
            if self.state == "walking" and self.target_position:
                self.move_towards_target(self.target_position, speed_factor=0.8)
            elif self.state == "working":
                self.simulate_work_activity()
            # Break and idle states don't require specific actions
            
            # Occasionally print status for debugging
            if int(self.robot.getTime()) % 30 == 0 and self.robot.getTime() > 1:
                status = self.get_worker_status()
                print(f"Worker {self.worker_id}: {status['state']} at {status['position'][:2]}")

def main():
    """Main function to run the human worker controller"""
    # Get worker ID from command line arguments
    if len(sys.argv) > 1:
        worker_id = sys.argv[1]
    else:
        worker_id = "worker1"  # Default
    
    # Create and run worker
    worker = HumanWorker(worker_id)
    worker.run()

if __name__ == "__main__":
    main() 