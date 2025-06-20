#!/usr/bin/env python3
"""
Webots Robot Controller for Unitree Go2 Behavioral Study
Integrates with the complete behavioral study system
"""

import sys
import os
import math
import numpy as np
from controller import Robot, Camera, RangeFinder, GPS, Compass, InertialUnit, Lidar

# Add the current directory to path to import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from behavioral_control import BehaviorController, BehaviorParameters
    from data_collection import DataCollectionSystem
    from astar_pathfinding import AStarPlanner
    from ConvertTo2_5D import EnvironmentMapper, GridMapConfig, SensorData
except ImportError as e:
    print(f"Warning: Could not import behavioral study modules: {e}")
    print("Running in basic navigation mode only")

class WebotsGo2Controller:
    """
    Webots controller for Unitree Go2 robot with behavioral study integration
    """
    
    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize sensors
        self._initialize_sensors()
        
        # Initialize behavioral study components if available
        self._initialize_behavioral_system()
        
        # Robot state
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0]
        self.velocity = [0, 0, 0]
        
        # Worker detection
        self.detected_workers = {}
        self.worker_detection_range = 10.0  # meters
        
        # Navigation
        self.current_target = None
        self.path = []
        self.path_index = 0
        
        print("Webots Go2 Controller initialized")
    
    def _initialize_sensors(self):
        """Initialize all robot sensors"""
        try:
            # Camera (RealSense RGB)
            self.camera = self.robot.getDevice("realsense_rgb")
            if self.camera:
                self.camera.enable(self.timestep)
                print("✓ RGB Camera initialized")
            
            # Depth camera (RealSense Depth)
            self.depth_camera = self.robot.getDevice("realsense_depth")
            if self.depth_camera:
                self.depth_camera.enable(self.timestep)
                print("✓ Depth Camera initialized")
            
            # GPS
            self.gps = self.robot.getDevice("gps")
            if self.gps:
                self.gps.enable(self.timestep)
                print("✓ GPS initialized")
            
            # Compass
            self.compass = self.robot.getDevice("compass")
            if self.compass:
                self.compass.enable(self.timestep)
                print("✓ Compass initialized")
            
            # IMU
            self.imu = self.robot.getDevice("imu")
            if self.imu:
                self.imu.enable(self.timestep)
                print("✓ IMU initialized")
            
            # Lidar
            self.lidar = self.robot.getDevice("lidar")
            if self.lidar:
                self.lidar.enable(self.timestep)
                print("✓ Lidar initialized")
            
        except Exception as e:
            print(f"Warning: Error initializing sensors: {e}")
    
    def _initialize_behavioral_system(self):
        """Initialize behavioral study system"""
        self.behavioral_system_available = False
        
        try:
            # Initialize behavioral parameters
            behavior_params = BehaviorParameters(
                patrol_speed=1.0,
                approach_speed=0.5,
                observe_speed=0.2,
                retreat_speed=0.8,
                close_threshold=2.0,
                very_close_threshold=1.0
            )
            
            # Initialize behavioral controller
            self.behavior_controller = BehaviorController(behavior_params)
            
            # Initialize data collection system
            session_id = f"webots_session_{int(self.robot.getTime())}"
            self.data_collector = DataCollectionSystem(session_id, "webots_participant")
            
            # Initialize environment mapping
            map_config = GridMapConfig(
                resolution=0.1,
                width=400,  # 40m x 40m area
                height=400,
                max_height=5.0,
                min_height=-1.0
            )
            self.environment_mapper = EnvironmentMapper(map_config, world_origin=(-20, -20))
            
            # Initialize path planner
            self.path_planner = AStarPlanner(
                grid_resolution=0.1,
                max_slope=0.3,
                max_step_height=0.15,
                robot_radius=0.3
            )
            
            self.behavioral_system_available = True
            print("✓ Behavioral study system initialized")
            
        except Exception as e:
            print(f"Warning: Behavioral system not available: {e}")
            self.behavioral_system_available = False
    
    def get_sensor_data(self):
        """Collect data from all sensors"""
        sensor_data = {}
        
        try:
            # GPS position
            if self.gps:
                gps_values = self.gps.getValues()
                self.position = [gps_values[0], gps_values[1], gps_values[2]]
                sensor_data['gps'] = self.position
            
            # Compass orientation
            if self.compass:
                compass_values = self.compass.getValues()
                # Convert to euler angles
                heading = math.atan2(compass_values[1], compass_values[0])
                self.orientation = [0, 0, heading]  # Simplified - only yaw
                sensor_data['compass'] = compass_values
            
            # IMU data
            if self.imu:
                imu_values = self.imu.getRollPitchYaw()
                sensor_data['imu'] = imu_values
                self.orientation = [imu_values[0], imu_values[1], imu_values[2]]
            
            # Camera image
            if self.camera:
                rgb_image = self.camera.getImageArray()
                if rgb_image:
                    sensor_data['rgb_image'] = np.array(rgb_image, dtype=np.uint8)
            
            # Depth image
            if self.depth_camera:
                depth_image = self.depth_camera.getRangeImageArray()
                if depth_image:
                    sensor_data['depth_image'] = np.array(depth_image, dtype=np.float32)
            
            # Lidar data
            if self.lidar:
                lidar_data = self.lidar.getRangeImage()
                if lidar_data:
                    sensor_data['lidar'] = np.array(lidar_data)
            
        except Exception as e:
            print(f"Error collecting sensor data: {e}")
        
        return sensor_data
    
    def detect_workers(self):
        """Detect human workers in the environment using sensor data"""
        detected_workers = []
        
        try:
            # Get current robot position
            robot_pos = self.position
            
            # Simulate worker detection using known worker positions
            # In a real implementation, this would use computer vision
            # For simulation, we'll check for nearby "Worker" robots
            
            # Get supervisor to access other robots (if available)
            supervisor = self.robot.getSupervisor()
            if supervisor:
                # Find worker robots
                for i in range(1, 4):  # Workers 1-3
                    worker_name = f"Worker{i}"
                    worker_node = supervisor.getFromDef(worker_name)
                    if worker_node:
                        worker_pos = worker_node.getPosition()
                        distance = math.sqrt(
                            (worker_pos[0] - robot_pos[0])**2 + 
                            (worker_pos[1] - robot_pos[1])**2
                        )
                        
                        if distance <= self.worker_detection_range:
                            detected_workers.append((i, worker_pos[0], worker_pos[1], worker_pos[2]))
            
            # Fallback: simulate workers at known approximate locations
            if not detected_workers:
                simulated_workers = [
                    (1, -5, -3, 0),
                    (2, 8, 5, 0),
                    (3, 3, -8, 0)
                ]
                
                for worker_id, x, y, z in simulated_workers:
                    distance = math.sqrt((x - robot_pos[0])**2 + (y - robot_pos[1])**2)
                    if distance <= self.worker_detection_range:
                        # Add some noise to simulate detection uncertainty
                        noisy_x = x + np.random.normal(0, 0.5)
                        noisy_y = y + np.random.normal(0, 0.5)
                        detected_workers.append((worker_id, noisy_x, noisy_y, z))
        
        except Exception as e:
            print(f"Error in worker detection: {e}")
        
        return detected_workers
    
    def update_environment_map(self, sensor_data):
        """Update the environment map with new sensor data"""
        if not self.behavioral_system_available:
            return
        
        try:
            if 'rgb_image' in sensor_data and 'depth_image' in sensor_data:
                # Create SensorData object
                rgb_img = sensor_data['rgb_image']
                depth_img = sensor_data['depth_image']
                
                # Robot pose (x, y, z, roll, pitch, yaw)
                robot_pose = (
                    self.position[0], self.position[1], self.position[2],
                    self.orientation[0], self.orientation[1], self.orientation[2]
                )
                
                sensor_data_obj = SensorData(rgb_img, depth_img, robot_pose, self.robot.getTime())
                
                # Process sensor data
                self.environment_mapper.process_sensor_data(sensor_data_obj)
                
                # Update path planner with new map
                height_map, obstacle_map = self.environment_mapper.get_maps_for_pathfinding()
                self.path_planner.set_environment(height_map, obstacle_map)
        
        except Exception as e:
            print(f"Error updating environment map: {e}")
    
    def plan_path_to_target(self, target_position):
        """Plan a path to the target position"""
        if not self.behavioral_system_available:
            return None
        
        try:
            # Convert world coordinates to grid coordinates
            start_grid = self.environment_mapper.world_to_grid(self.position[0], self.position[1])
            goal_grid = self.environment_mapper.world_to_grid(target_position[0], target_position[1])
            
            # Find path
            path = self.path_planner.find_path(start_grid, goal_grid)
            
            if path:
                # Convert back to world coordinates
                world_path = self.path_planner.grid_to_world(path, self.environment_mapper.world_origin)
                return world_path
            
        except Exception as e:
            print(f"Error planning path: {e}")
        
        return None
    
    def execute_movement(self, target_position, speed):
        """Execute movement towards target position"""
        try:
            # Calculate direction to target
            dx = target_position[0] - self.position[0]
            dy = target_position[1] - self.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.5:  # Close enough to target
                return True
            
            # Calculate desired heading
            desired_heading = math.atan2(dy, dx)
            current_heading = self.orientation[2]
            
            # Simple movement simulation (in real implementation, would control motors)
            print(f"Moving towards ({target_position[0]:.2f}, {target_position[1]:.2f}) at speed {speed:.2f}")
            
            # Update velocity for data logging
            if distance > 0:
                self.velocity = [
                    (dx / distance) * speed,
                    (dy / distance) * speed,
                    0
                ]
            
            return False
            
        except Exception as e:
            print(f"Error executing movement: {e}")
            return True
    
    def log_telemetry_data(self, behavioral_result, detected_workers):
        """Log robot telemetry data"""
        if not self.behavioral_system_available:
            return
        
        try:
            # Calculate worker distances
            worker_distances = {}
            for worker_id, x, y, z in detected_workers:
                distance = math.sqrt((x - self.position[0])**2 + (y - self.position[1])**2)
                worker_distances[worker_id] = distance
            
            # Create telemetry data
            robot_state = {
                'position': tuple(self.position),
                'orientation': tuple(self.orientation),
                'velocity': tuple(self.velocity),
                'behavioral_state': behavioral_result.get('current_state', 'unknown'),
                'target_worker_id': self.behavior_controller.state_machine.target_worker_id,
                'worker_distances': worker_distances,
                'movement_speed': behavioral_result.get('speed', 0.0),
                'emergency_status': behavioral_result.get('emergency_stop', False)
            }
            
            # Log to data collection system
            self.data_collector.collect_robot_data(robot_state)
            
            # Track worker interactions
            for worker_id, distance in worker_distances.items():
                self.data_collector.track_worker_interaction(
                    worker_id, distance, robot_state['behavioral_state']
                )
        
        except Exception as e:
            print(f"Error logging telemetry: {e}")
    
    def run(self):
        """Main control loop"""
        print("Starting Webots Go2 robot control loop")
        
        # Start data collection if available
        if self.behavioral_system_available:
            self.data_collector.start_collection()
        
        step_count = 0
        
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            
            try:
                # Collect sensor data
                sensor_data = self.get_sensor_data()
                
                # Detect workers
                detected_workers = self.detect_workers()
                
                # Update environment map periodically
                if step_count % 10 == 0:  # Every 10 steps
                    self.update_environment_map(sensor_data)
                
                # Update behavioral system if available
                if self.behavioral_system_available:
                    # Update behavior controller with worker detections
                    self.behavior_controller.update_worker_detections(detected_workers)
                    
                    # Get behavioral decision
                    behavioral_result = self.behavior_controller.update()
                    
                    # Get target position from behavioral controller
                    target_pos = behavioral_result.get('target_position')
                    if target_pos:
                        # Plan path if needed
                        if not self.path or step_count % 50 == 0:  # Replan every 50 steps
                            self.path = self.plan_path_to_target(target_pos)
                            self.path_index = 0
                        
                        # Execute movement
                        if self.path and self.path_index < len(self.path):
                            current_waypoint = self.path[self.path_index]
                            reached = self.execute_movement(current_waypoint, behavioral_result.get('speed', 0.5))
                            if reached:
                                self.path_index += 1
                        else:
                            # Move directly to target
                            self.execute_movement(target_pos, behavioral_result.get('speed', 0.5))
                    
                    # Log telemetry data
                    if step_count % 5 == 0:  # Every 5 steps
                        self.log_telemetry_data(behavioral_result, detected_workers)
                
                # Print status periodically
                if step_count % 100 == 0:
                    print(f"Robot at ({self.position[0]:.2f}, {self.position[1]:.2f}), "
                          f"Workers detected: {len(detected_workers)}")
                    
                    if self.behavioral_system_available:
                        current_state = self.behavior_controller.state_machine.current_state.value
                        print(f"Behavioral state: {current_state}")
            
            except Exception as e:
                print(f"Error in main loop: {e}")
                continue
        
        # Stop data collection
        if self.behavioral_system_available:
            self.data_collector.stop_collection()
        
        print("Robot control loop ended")

def main():
    """Main function"""
    controller = WebotsGo2Controller()
    controller.run()

if __name__ == "__main__":
    main() 