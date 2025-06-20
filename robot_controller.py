#!/usr/bin/env python3
"""
Unitree Go2 Robot Controller for Webots Construction Environment
Integrates A* pathfinding with 2.5D environment mapping
"""

import sys
import numpy as np
import math
from typing import List, Tuple, Optional
from controller import Robot, Camera, RangeFinder, GPS, Compass, Motor

# Import our custom modules
try:
    from ConvertTo2_5D import EnvironmentMapper, GridMapConfig, SensorData, WebotsIntegration
    import importlib
    astar_module = importlib.import_module("A*")
    AStarPlanner = astar_module.AStarPlanner
except ImportError as e:
    print(f"Import error: {e}")
    print("Note: This controller requires ConvertTo2_5D.py and A*.py modules")
    import sys
    sys.exit(1)

class Go2Controller:
    """
    Main controller for Unitree Go2 robot in Webots construction environment
    """
    
    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize devices
        self._initialize_sensors()
        self._initialize_motors()
        
        # Initialize navigation system
        self._initialize_navigation()
        
        # Robot state
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0
        self.navigation_active = False
        self.current_waypoint_index = 0
        self.waypoints = []
        
        # Control parameters
        self.max_speed = 2.0  # m/s
        self.turn_speed = 1.0  # rad/s
        self.position_tolerance = 0.2  # meters
        self.orientation_tolerance = 0.1  # radians
        
        print("Go2 Controller initialized successfully!")
    
    def _initialize_sensors(self):
        """Initialize all sensors"""
        try:
            # Camera (RealSense D435i RGB)
            self.camera = self.robot.getDevice('camera')
            self.camera.enable(self.timestep)
            
            # Range finder (RealSense D435i depth)
            self.range_finder = self.robot.getDevice('range_finder')
            self.range_finder.enable(self.timestep)
            
            # GPS for position
            self.gps = self.robot.getDevice('gps')
            self.gps.enable(self.timestep)
            
            # Compass for orientation
            self.compass = self.robot.getDevice('compass')
            self.compass.enable(self.timestep)
            
            # IMU (if available)
            try:
                self.imu = self.robot.getDevice('imu')
                self.imu.enable(self.timestep)
            except:
                self.imu = None
                print("IMU not available")
                
        except Exception as e:
            print(f"Error initializing sensors: {e}")
    
    def _initialize_motors(self):
        """Initialize motor devices for Go2 robot"""
        try:
            # Get motor devices (assuming standard Go2 naming)
            motor_names = [
                'front_left_hip', 'front_left_thigh', 'front_left_calf',
                'front_right_hip', 'front_right_thigh', 'front_right_calf',
                'rear_left_hip', 'rear_left_thigh', 'rear_left_calf',
                'rear_right_hip', 'rear_right_thigh', 'rear_right_calf'
            ]
            
            self.motors = {}
            for name in motor_names:
                try:
                    motor = self.robot.getDevice(name)
                    motor.setPosition(float('inf'))  # Set to velocity control
                    motor.setVelocity(0.0)
                    self.motors[name] = motor
                except:
                    print(f"Motor {name} not found")
            
            print(f"Initialized {len(self.motors)} motors")
            
        except Exception as e:
            print(f"Error initializing motors: {e}")
    
    def _initialize_navigation(self):
        """Initialize navigation and mapping systems"""
        # Configure mapping
        self.map_config = GridMapConfig(
            resolution=0.1,  # 10cm resolution
            width=1000,      # 100m x 100m area
            height=1000,
            max_height=5.0,
            min_height=-2.0,
            robot_height=0.3,
            traversability_threshold=0.3
        )
        
        # Initialize environment mapper
        self.mapper = EnvironmentMapper(self.map_config, world_origin=(-50.0, -50.0))
        
        # Initialize A* planner
        self.planner = AStarPlanner(
            grid_resolution=0.1,
            max_slope=0.3,
            max_step_height=0.15,
            robot_radius=0.3
        )
        
        # Webots integration
        self.webots_integration = WebotsIntegration(self.mapper)
        self.webots_integration.initialize_webots_devices(self.robot)
        
        print("Navigation system initialized")
    
    def get_sensor_data(self) -> SensorData:
        """Get current sensor data from robot"""
        try:
            # Get RGB image
            rgb_data = self.camera.getImage()
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            rgb_image = np.frombuffer(rgb_data, np.uint8).reshape((height, width, 4))[:,:,:3]
            
            # Get depth image
            depth_data = self.range_finder.getRangeImage()
            depth_width = self.range_finder.getWidth()
            depth_height = self.range_finder.getHeight()
            depth_image = np.array(depth_data).reshape((depth_height, depth_width))
            
            # Get robot pose
            camera_pose = self.get_robot_pose()
            
            # Create sensor data object
            return SensorData(
                rgb_image=rgb_image,
                depth_image=depth_image,
                camera_pose=camera_pose,
                timestamp=self.robot.getTime()
            )
            
        except Exception as e:
            print(f"Error getting sensor data: {e}")
            # Return dummy data
            return SensorData(
                rgb_image=np.zeros((480, 640, 3), dtype=np.uint8),
                depth_image=np.ones((480, 640), dtype=np.float32),
                camera_pose=(0, 0, 0.3, 0, 0, 0),
                timestamp=self.robot.getTime()
            )
    
    def get_robot_pose(self) -> Tuple[float, float, float, float, float, float]:
        """Get current robot pose"""
        try:
            # Get position from GPS
            position = self.gps.getValues()
            x, y, z = position[0], position[1], position[2]
            
            # Get orientation from compass
            compass_values = self.compass.getValues()
            
            # Calculate yaw from compass
            yaw = math.atan2(compass_values[0], compass_values[1])
            
            # For simplicity, assume roll and pitch are small
            roll, pitch = 0.0, 0.0
            
            self.current_position = [x, y]
            self.current_orientation = yaw
            
            return (x, y, z, roll, pitch, yaw)
            
        except Exception as e:
            print(f"Error getting robot pose: {e}")
            return (0.0, 0.0, 0.3, 0.0, 0.0, 0.0)
    
    def set_walking_gait(self, forward_speed: float, lateral_speed: float, turn_speed: float):
        """
        Set walking gait for the robot
        Simplified gait implementation - in practice, this would be much more complex
        """
        try:
            # This is a very simplified gait implementation
            # Real implementation would require proper leg coordination and balance
            
            # Scale speeds
            forward_velocity = forward_speed * self.max_speed
            turn_velocity = turn_speed * self.turn_speed
            
            # Simple velocity mapping to leg movements
            # This is a placeholder - real Go2 control would be much more sophisticated
            base_frequency = 2.0  # Hz
            amplitude = 0.5
            
            time = self.robot.getTime()
            phase_offset = math.pi / 2
            
            # Generate basic walking pattern
            for i, motor_name in enumerate(self.motors.keys()):
                if 'hip' in motor_name:
                    # Hip motors for turning
                    velocity = turn_velocity * 0.5
                elif 'thigh' in motor_name:
                    # Thigh motors for forward motion
                    phase = (i % 4) * phase_offset
                    velocity = forward_velocity * math.sin(2 * math.pi * base_frequency * time + phase)
                elif 'calf' in motor_name:
                    # Calf motors for ground clearance
                    phase = (i % 4) * phase_offset
                    velocity = amplitude * math.cos(2 * math.pi * base_frequency * time + phase)
                else:
                    velocity = 0.0
                
                self.motors[motor_name].setVelocity(velocity)
                
        except Exception as e:
            print(f"Error setting walking gait: {e}")
    
    def stop_robot(self):
        """Stop all robot movement"""
        try:
            for motor in self.motors.values():
                motor.setVelocity(0.0)
        except Exception as e:
            print(f"Error stopping robot: {e}")
    
    def navigate_to_goal(self, goal_x: float, goal_y: float) -> bool:
        """
        Navigate to a specific goal position using A* pathfinding
        
        Args:
            goal_x, goal_y: Goal position in world coordinates
            
        Returns:
            True if path found and navigation started, False otherwise
        """
        try:
            # Update environment map with current sensor data
            sensor_data = self.get_sensor_data()
            self.mapper.process_sensor_data(sensor_data)
            
            # Get maps for pathfinding
            height_map, obstacle_map = self.mapper.get_maps_for_pathfinding()
            self.planner.set_environment(height_map, obstacle_map)
            
            # Convert world coordinates to grid coordinates
            start_grid = self.mapper.world_to_grid(self.current_position[0], self.current_position[1])
            goal_grid = self.mapper.world_to_grid(goal_x, goal_y)
            
            print(f"Planning path from {start_grid} to {goal_grid}")
            
            # Find path
            path = self.planner.find_path(start_grid, goal_grid)
            
            if path is None:
                print("No path found to goal!")
                return False
            
            # Convert path back to world coordinates
            world_path = self.planner.grid_to_world(path, self.mapper.world_origin)
            
            # Smooth path
            smoothed_path = self.planner.smooth_path(path)
            world_path_smoothed = self.planner.grid_to_world(smoothed_path, self.mapper.world_origin)
            
            self.waypoints = world_path_smoothed
            self.current_waypoint_index = 0
            self.navigation_active = True
            
            print(f"Path found with {len(self.waypoints)} waypoints")
            return True
            
        except Exception as e:
            print(f"Error in navigation planning: {e}")
            return False
    
    def update_navigation(self):
        """Update navigation controller"""
        if not self.navigation_active or not self.waypoints:
            return
            
        try:
            # Check if we've reached the current waypoint
            if self.current_waypoint_index >= len(self.waypoints):
                print("Goal reached!")
                self.stop_robot()
                self.navigation_active = False
                return
            
            # Get current target waypoint
            target_x, target_y, target_z = self.waypoints[self.current_waypoint_index]
            
            # Calculate distance to target
            dx = target_x - self.current_position[0]
            dy = target_y - self.current_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if we've reached the waypoint
            if distance < self.position_tolerance:
                print(f"Reached waypoint {self.current_waypoint_index}")
                self.current_waypoint_index += 1
                return
            
            # Calculate desired heading
            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - self.current_orientation
            
            # Normalize angle
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Simple proportional controller
            if abs(heading_error) > self.orientation_tolerance:
                # Turn towards target
                turn_speed = np.clip(heading_error * 2.0, -1.0, 1.0)
                self.set_walking_gait(0.0, 0.0, turn_speed)
            else:
                # Move forward
                forward_speed = np.clip(distance * 2.0, 0.0, 1.0)
                self.set_walking_gait(forward_speed, 0.0, 0.0)
                
        except Exception as e:
            print(f"Error in navigation update: {e}")
    
    def update_dynamic_obstacles(self):
        """Update dynamic obstacle detection"""
        try:
            # Get current sensor data
            sensor_data = self.get_sensor_data()
            
            # Store previous occupancy for comparison
            previous_occupancy = self.mapper.occupancy_map.copy()
            
            # Detect dynamic obstacles
            dynamic_obstacles = self.mapper.detect_dynamic_obstacles(sensor_data, previous_occupancy)
            
            if dynamic_obstacles:
                print(f"Detected {len(dynamic_obstacles)} dynamic obstacles")
                
                # Add to planner's dynamic obstacles
                self.planner.clear_dynamic_obstacles()
                for obs_x, obs_y in dynamic_obstacles:
                    self.planner.add_dynamic_obstacle(obs_x, obs_y, radius=3)
                
                # Replan if navigating
                if self.navigation_active and self.waypoints:
                    current_goal = self.waypoints[-1]
                    print("Replanning due to dynamic obstacles...")
                    self.navigate_to_goal(current_goal[0], current_goal[1])
                    
        except Exception as e:
            print(f"Error updating dynamic obstacles: {e}")
    
    def run(self):
        """Main control loop"""
        print("Starting Go2 robot controller...")
        
        # Example navigation goals (you can modify these)
        goals = [
            (10.0, 0.0),   # Move 10m forward
            (10.0, 10.0),  # Turn and move to corner
            (0.0, 10.0),   # Move to another corner
            (0.0, 0.0)     # Return to start
        ]
        
        current_goal_index = 0
        navigation_started = False
        last_update_time = 0
        
        while self.robot.step(self.timestep) != -1:
            current_time = self.robot.getTime()
            
            # Update robot pose
            self.get_robot_pose()
            
            # Start navigation to first goal
            if not navigation_started and current_time > 2.0:  # Wait 2 seconds before starting
                if current_goal_index < len(goals):
                    goal_x, goal_y = goals[current_goal_index]
                    print(f"Navigating to goal {current_goal_index}: ({goal_x}, {goal_y})")
                    
                    if self.navigate_to_goal(goal_x, goal_y):
                        navigation_started = True
                    else:
                        print("Failed to start navigation")
                        current_goal_index += 1
            
            # Update navigation
            if self.navigation_active:
                self.update_navigation()
            
            # Check if current goal is reached
            if navigation_started and not self.navigation_active:
                current_goal_index += 1
                navigation_started = False
                print(f"Goal {current_goal_index - 1} completed")
                
                if current_goal_index >= len(goals):
                    print("All goals completed!")
                    self.stop_robot()
                    break
            
            # Update dynamic obstacles periodically
            if current_time - last_update_time > 1.0:  # Update every second
                self.update_dynamic_obstacles()
                last_update_time = current_time
        
        print("Robot controller finished")

# Entry point for Webots controller
if __name__ == "__main__":
    controller = Go2Controller()
    controller.run() 