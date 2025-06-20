#!/usr/bin/env python3

"""
Enhanced Webots Go2 Robot Controller

This controller integrates with the official Webots Go2 proto and provides
comprehensive functionality including sensor data collection, navigation,
behavioral control, and human-robot interaction in the construction site.
"""

from controller import Robot, Camera, RangeFinder, Lidar, GPS, Compass, InertialUnit
import sys
import os
import math
import json
import time
import numpy as np

# Add project modules to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

try:
    from astar_pathfinding import AStarPathfinder
    from ConvertTo2_5D import EnvironmentMapper
    from behavioral_control import BehavioralController
    from data_collection import DataCollector
    from sensor_processing import SensorProcessor
except ImportError as e:
    print(f"Warning: Could not import some modules: {e}")
    print("Some functionality may be limited")

class Go2RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize sensors and actuators
        self.init_devices()
        
        # Initialize core systems
        self.init_systems()
        
        # Robot state
        self.current_position = [0, 0, 0]
        self.current_orientation = 0
        self.current_state = "idle"
        
        # Navigation and mapping
        self.environment_map = None
        self.current_path = []
        self.target_position = None
        
        # Human tracking
        self.detected_humans = {}
        self.last_human_detection = {}
        
        print("Go2 Robot Controller initialized successfully")
    
    def init_devices(self):
        """Initialize all sensors and actuators for the Go2 robot"""
        # Camera sensors
        try:
            self.front_camera = self.robot.getDevice('front_camera')
            if self.front_camera:
                self.front_camera.enable(self.timestep)
                print("Front camera initialized")
        except:
            print("Warning: Front camera not found")
            self.front_camera = None
        
        # Depth camera (RangeFinder)
        try:
            self.depth_camera = self.robot.getDevice('depth_camera')
            if self.depth_camera:
                self.depth_camera.enable(self.timestep)
                print("Depth camera initialized")
        except:
            print("Warning: Depth camera not found")
            self.depth_camera = None
        
        # Lidar
        try:
            self.lidar = self.robot.getDevice('lidar')
            if self.lidar:
                self.lidar.enable(self.timestep)
                self.lidar.enablePointCloud()
                print("Lidar initialized")
        except:
            print("Warning: Lidar not found")
            self.lidar = None
        
        # GPS for position tracking
        try:
            self.gps = self.robot.getDevice('gps')
            if self.gps:
                self.gps.enable(self.timestep)
                print("GPS initialized")
        except:
            print("Warning: GPS not found")
            self.gps = None
        
        # Compass for orientation
        try:
            self.compass = self.robot.getDevice('compass')
            if self.compass:
                self.compass.enable(self.timestep)
                print("Compass initialized")
        except:
            print("Warning: Compass not found")
            self.compass = None
        
        # IMU for inertial measurements
        try:
            self.imu = self.robot.getDevice('imu')
            if self.imu:
                self.imu.enable(self.timestep)
                print("IMU initialized")
        except:
            print("Warning: IMU not found")
            self.imu = None
        
        # Motors (for Go2 proto, these might be named differently)
        self.motors = {}
        motor_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        
        for name in motor_names:
            try:
                motor = self.robot.getDevice(name)
                if motor:
                    self.motors[name] = motor
                    motor.setPosition(0.0)  # Set to neutral position
            except:
                print(f"Warning: Motor {name} not found")
        
        print(f"Initialized {len(self.motors)} motors")
    
    def init_systems(self):
        """Initialize the robot's core systems"""
        try:
            # Environment mapper
            self.environment_mapper = EnvironmentMapper(
                grid_resolution=0.1,
                environment_size=(50, 50)
            )
            
            # Path planner
            self.pathfinder = AStarPathfinder(
                grid_resolution=0.1,
                robot_radius=0.3,
                max_slope=30,
                max_step_height=0.15
            )
            
            # Behavioral controller
            self.behavioral_controller = BehavioralController()
            
            # Data collector
            self.data_collector = DataCollector(
                session_name=f"webots_simulation_{int(time.time())}"
            )
            
            # Sensor processor
            self.sensor_processor = SensorProcessor()
            
            print("All core systems initialized")
            
        except Exception as e:
            print(f"Warning: Could not initialize all systems: {e}")
            # Initialize minimal systems
            self.environment_mapper = None
            self.pathfinder = None
            self.behavioral_controller = None
            self.data_collector = None
            self.sensor_processor = None
    
    def update_sensors(self):
        """Update all sensor readings"""
        sensor_data = {}
        
        # GPS position
        if self.gps:
            try:
                gps_values = self.gps.getValues()
                self.current_position = list(gps_values)
                sensor_data['gps'] = gps_values
            except:
                pass
        
        # Compass orientation
        if self.compass:
            try:
                north = self.compass.getValues()
                self.current_orientation = math.atan2(north[1], north[0])
                sensor_data['compass'] = self.current_orientation
            except:
                pass
        
        # IMU data
        if self.imu:
            try:
                roll_pitch_yaw = self.imu.getRollPitchYaw()
                sensor_data['imu'] = {
                    'roll': roll_pitch_yaw[0],
                    'pitch': roll_pitch_yaw[1],
                    'yaw': roll_pitch_yaw[2]
                }
            except:
                pass
        
        # Camera image
        if self.front_camera:
            try:
                image = self.front_camera.getImage()
                if image:
                    width = self.front_camera.getWidth()
                    height = self.front_camera.getHeight()
                    sensor_data['camera'] = {
                        'width': width,
                        'height': height,
                        'image_available': True
                    }
            except:
                pass
        
        # Depth data
        if self.depth_camera:
            try:
                range_image = self.depth_camera.getRangeImage()
                if range_image:
                    sensor_data['depth'] = {
                        'width': self.depth_camera.getWidth(),
                        'height': self.depth_camera.getHeight(),
                        'max_range': self.depth_camera.getMaxRange(),
                        'data_available': True
                    }
            except:
                pass
        
        # Lidar data
        if self.lidar:
            try:
                point_cloud = self.lidar.getPointCloud()
                if point_cloud:
                    sensor_data['lidar'] = {
                        'points': len(point_cloud) // 3,  # Each point has x,y,z
                        'horizontal_resolution': self.lidar.getHorizontalResolution(),
                        'number_of_layers': self.lidar.getNumberOfLayers(),
                        'data_available': True
                    }
            except:
                pass
        
        return sensor_data
    
    def detect_humans(self, sensor_data):
        """Detect humans in the environment using available sensors"""
        detected_humans = []
        
        # Simple human detection based on Lidar data
        if self.lidar and 'lidar' in sensor_data and sensor_data['lidar']['data_available']:
            try:
                point_cloud = self.lidar.getPointCloud()
                if point_cloud:
                    # Process point cloud to detect human-like objects
                    # This is a simplified detection algorithm
                    for i in range(0, len(point_cloud), 3):
                        x, y, z = point_cloud[i], point_cloud[i+1], point_cloud[i+2]
                        
                        # Look for objects at human height (1.5-2.0m) and reasonable distance
                        if 1.2 < z < 2.2 and math.sqrt(x*x + y*y) < 15:
                            # Convert to global coordinates
                            global_x = self.current_position[0] + x * math.cos(self.current_orientation) - y * math.sin(self.current_orientation)
                            global_y = self.current_position[1] + x * math.sin(self.current_orientation) + y * math.cos(self.current_orientation)
                            
                            detected_humans.append({
                                'position': [global_x, global_y, z],
                                'distance': math.sqrt(x*x + y*y),
                                'confidence': 0.7  # Simple confidence score
                            })
            except Exception as e:
                print(f"Error in human detection: {e}")
        
        # Update human tracking
        current_time = self.robot.getTime()
        for human in detected_humans:
            human_id = f"human_{len(self.detected_humans)}"
            self.detected_humans[human_id] = human
            self.last_human_detection[human_id] = current_time
        
        return detected_humans
    
    def update_environment_map(self, sensor_data):
        """Update the environment map with new sensor data"""
        if not self.environment_mapper:
            return
        
        try:
            # Process Lidar data for mapping
            if self.lidar and 'lidar' in sensor_data:
                point_cloud = self.lidar.getPointCloud()
                if point_cloud:
                    # Convert point cloud to occupancy grid updates
                    robot_pos = self.current_position[:2]  # x, y only
                    robot_orientation = self.current_orientation
                    
                    self.environment_mapper.update_from_lidar(
                        point_cloud, robot_pos, robot_orientation
                    )
            
            # Process depth camera data
            if self.depth_camera and 'depth' in sensor_data:
                range_image = self.depth_camera.getRangeImage()
                if range_image:
                    self.environment_mapper.update_from_depth_camera(
                        range_image,
                        self.current_position[:2],
                        self.current_orientation
                    )
                    
        except Exception as e:
            print(f"Error updating environment map: {e}")
    
    def plan_path(self, target):
        """Plan a path to the target position"""
        if not self.pathfinder or not self.environment_mapper:
            return []
        
        try:
            # Get current environment map
            occupancy_grid = self.environment_mapper.get_occupancy_grid()
            
            # Plan path
            start = self.current_position[:2]
            path = self.pathfinder.find_path(start, target, occupancy_grid)
            
            if path:
                self.current_path = path
                self.target_position = target
                print(f"Path planned with {len(path)} waypoints")
                return path
            else:
                print("No path found to target")
                return []
                
        except Exception as e:
            print(f"Error in path planning: {e}")
            return []
    
    def execute_movement(self):
        """Execute movement along the planned path"""
        if not self.current_path:
            return
        
        try:
            # Simple movement execution for Go2
            # In a real implementation, this would use the Go2's locomotion system
            
            current_pos = self.current_position[:2]
            next_waypoint = self.current_path[0]
            
            # Calculate direction to next waypoint
            dx = next_waypoint[0] - current_pos[0]
            dy = next_waypoint[1] - current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.5:  # Close enough to waypoint
                self.current_path.pop(0)  # Remove reached waypoint
                if not self.current_path:
                    print("Target reached!")
                    self.current_state = "idle"
            else:
                # Move towards waypoint
                self.current_state = "moving"
                
                # Simple motor control (this would be much more complex for real Go2)
                # For now, we'll just track the intended movement
                print(f"Moving towards waypoint {next_waypoint}, distance: {distance:.2f}m")
                
        except Exception as e:
            print(f"Error in movement execution: {e}")
    
    def collect_telemetry_data(self, sensor_data):
        """Collect telemetry data for analysis"""
        if not self.data_collector:
            return
        
        try:
            telemetry = {
                'timestamp': self.robot.getTime(),
                'position': self.current_position,
                'orientation': self.current_orientation,
                'state': self.current_state,
                'sensor_data': sensor_data,
                'detected_humans': len(self.detected_humans),
                'path_length': len(self.current_path)
            }
            
            self.data_collector.log_robot_telemetry(telemetry)
            
        except Exception as e:
            print(f"Error collecting telemetry: {e}")
    
    def run_behavioral_control(self, sensor_data):
        """Run behavioral control system"""
        if not self.behavioral_controller:
            return
        
        try:
            # Update behavioral parameters based on environment
            humans_nearby = [h for h in self.detected_humans.values() if h['distance'] < 5.0]
            
            if humans_nearby:
                # Adjust behavior when humans are nearby
                self.behavioral_controller.set_parameter('proximity_threshold', 2.0)
                self.behavioral_controller.set_parameter('movement_speed', 0.3)
            else:
                # Normal behavior
                self.behavioral_controller.set_parameter('proximity_threshold', 1.0)
                self.behavioral_controller.set_parameter('movement_speed', 0.5)
            
            # Update behavioral state
            self.behavioral_controller.update_state(sensor_data)
            
        except Exception as e:
            print(f"Error in behavioral control: {e}")
    
    def run(self):
        """Main control loop"""
        print("Go2 Robot starting main control loop")
        
        step_count = 0
        last_status_time = 0
        
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            current_time = self.robot.getTime()
            
            # Update sensors
            sensor_data = self.update_sensors()
            
            # Detect humans
            detected_humans = self.detect_humans(sensor_data)
            
            # Update environment map
            self.update_environment_map(sensor_data)
            
            # Run behavioral control
            self.run_behavioral_control(sensor_data)
            
            # Execute movement if path exists
            self.execute_movement()
            
            # Collect telemetry data
            self.collect_telemetry_data(sensor_data)
            
            # Periodic status updates
            if current_time - last_status_time > 5.0:  # Every 5 seconds
                print(f"Robot Status - Pos: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}], "
                      f"State: {self.current_state}, Humans detected: {len(detected_humans)}")
                last_status_time = current_time
            
            # Example: Set a patrol target every 30 seconds
            if step_count % (30000 // self.timestep) == 0:
                # Generate random patrol target within construction site
                target_x = np.random.uniform(-20, 20)
                target_y = np.random.uniform(-20, 20)
                self.plan_path([target_x, target_y])

def main():
    """Main function"""
    controller = Go2RobotController()
    controller.run()

if __name__ == "__main__":
    main() 