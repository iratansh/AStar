#!/usr/bin/env python3
"""
2.5D Environment Mapping for Unitree Go2 Robot in Webots Construction Site
Converts RealSense D435i sensor data to navigation-ready grid maps
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass
import math
import json

def griddata(points, values, xi, method='linear', fill_value=np.nan):
    """Simple fallback implementation when scipy is not available"""
    if len(points) == 0:
        return np.full(xi[0].shape, fill_value)
    
    # Simple nearest neighbor interpolation
    result = np.full(xi[0].shape, fill_value)
    for i in range(xi[0].shape[0]):
        for j in range(xi[0].shape[1]):
            min_dist = float('inf')
            nearest_value = fill_value
            
            for k, (py, px) in enumerate(points):
                dist = (i - py)**2 + (j - px)**2
                if dist < min_dist:
                    min_dist = dist
                    nearest_value = values[k]
            
            if min_dist < 100:  # Reasonable threshold
                result[i, j] = nearest_value
    
    return result

@dataclass
class SensorData:
    """Container for RealSense D435i sensor data"""
    rgb_image: np.ndarray
    depth_image: np.ndarray
    camera_pose: Tuple[float, float, float, float, float, float]  # x, y, z, roll, pitch, yaw
    timestamp: float

@dataclass
class GridMapConfig:
    """Configuration for 2.5D grid mapping"""
    resolution: float = 0.1  # meters per cell
    width: int = 500  # grid width in cells
    height: int = 500  # grid height in cells
    max_height: float = 3.0  # maximum height to consider
    min_height: float = -1.0  # minimum height to consider
    robot_height: float = 0.3  # robot body height
    traversability_threshold: float = 0.3  # max slope for traversability

class EnvironmentMapper:
    """
    Converts sensor data to 2.5D grid maps for navigation
    """
    
    def __init__(self, config: GridMapConfig, world_origin: Tuple[float, float] = (0.0, 0.0)):
        """
        Initialize environment mapper
        
        Args:
            config: Grid mapping configuration
            world_origin: (x, y) origin of the grid in world coordinates
        """
        self.config = config
        self.world_origin = world_origin
        
        # Initialize grid maps
        self.height_map = np.full((config.height, config.width), np.nan)
        self.occupancy_map = np.zeros((config.height, config.width), dtype=np.uint8)
        self.traversability_map = np.zeros((config.height, config.width), dtype=np.uint8)
        self.confidence_map = np.zeros((config.height, config.width), dtype=np.float32)
        
        # Camera intrinsics (RealSense D435i typical values)
        self.camera_intrinsics = {
            'fx': 383.0,  # focal length x
            'fy': 383.0,  # focal length y
            'cx': 320.0,  # principal point x
            'cy': 240.0,  # principal point y
            'width': 640,
            'height': 480
        }
        
        # Point cloud processing parameters
        self.max_range = 10.0  # maximum depth range in meters
        self.min_points_per_cell = 3  # minimum points for reliable height estimation
        
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices"""
        grid_x = int((world_x - self.world_origin[0]) / self.config.resolution)
        grid_y = int((world_y - self.world_origin[1]) / self.config.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates"""
        world_x = self.world_origin[0] + grid_x * self.config.resolution
        world_y = self.world_origin[1] + grid_y * self.config.resolution
        return world_x, world_y
    
    def depth_to_pointcloud(self, depth_image: np.ndarray, 
                           camera_pose: Tuple[float, float, float, float, float, float]) -> np.ndarray:
        """
        Convert depth image to 3D point cloud in world coordinates
        
        Args:
            depth_image: Depth image from RealSense
            camera_pose: Camera pose (x, y, z, roll, pitch, yaw)
            
        Returns:
            Point cloud as Nx3 array (x, y, z in world coordinates)
        """
        v, u = np.indices(depth_image.shape)
        
        # Normalize pixel coordinates
        x_normalized = (u - self.camera_intrinsics['cx']) / self.camera_intrinsics['fx']
        y_normalized = (v - self.camera_intrinsics['cy']) / self.camera_intrinsics['fy']
        
        # Deproject to 3D points in camera frame
        z_cam = depth_image
        x_cam = x_normalized * z_cam
        y_cam = y_normalized * z_cam
        
        # Filter out invalid depth points
        valid_mask = z_cam > 0
        
        # Combine into a single (N, 3) array of points
        point_cloud_cam = np.vstack((x_cam[valid_mask], y_cam[valid_mask], z_cam[valid_mask])).T
        
        return point_cloud_cam
    
    def transform_points(self, points: np.ndarray, 
                        pose: Tuple[float, float, float, float, float, float]) -> np.ndarray:
        """Transform points from camera frame to world frame"""
        x, y, z, roll, pitch, yaw = pose
        
        # Create rotation matrix
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        cos_pitch = np.cos(pitch)
        sin_pitch = np.sin(pitch)
        cos_roll = np.cos(roll)
        sin_roll = np.sin(roll)
        
        # Rotation matrix (ZYX order)
        R = np.array([
            [cos_yaw * cos_pitch, 
             cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
             cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
            [sin_yaw * cos_pitch,
             sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
             sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
            [-sin_pitch,
             cos_pitch * sin_roll,
             cos_pitch * cos_roll]
        ])
        
        # Apply rotation and translation
        points_rotated = points @ R.T
        points_world = points_rotated + np.array([x, y, z])
        
        return points_world
    
    def update_height_map(self, points: np.ndarray):
        """Update height map with new point cloud data"""
        if len(points) == 0:
            return
            
        # Convert points to grid coordinates
        grid_coords = []
        heights = []
        
        for point in points:
            world_x, world_y, world_z = point
            grid_x, grid_y = self.world_to_grid(world_x, world_y)
            
            # Check if within grid bounds
            if (0 <= grid_x < self.config.width and 
                0 <= grid_y < self.config.height and
                self.config.min_height <= world_z <= self.config.max_height):
                
                grid_coords.append((grid_x, grid_y))
                heights.append(world_z)
        
        if not grid_coords:
            return
            
        # Group points by grid cell and compute statistics
        cell_points = {}
        for (gx, gy), height in zip(grid_coords, heights):
            if (gx, gy) not in cell_points:
                cell_points[(gx, gy)] = []
            cell_points[(gx, gy)].append(height)
        
        # Update height map with median heights
        for (gx, gy), cell_heights in cell_points.items():
            if len(cell_heights) >= self.min_points_per_cell:
                median_height = np.median(cell_heights)
                confidence = min(len(cell_heights) / 10.0, 1.0)
                
                # Update with weighted average for stability
                if np.isnan(self.height_map[gy, gx]):
                    self.height_map[gy, gx] = median_height
                    self.confidence_map[gy, gx] = confidence
                else:
                    old_confidence = self.confidence_map[gy, gx]
                    total_confidence = old_confidence + confidence
                    
                    if total_confidence > 0:
                        self.height_map[gy, gx] = (
                            (self.height_map[gy, gx] * old_confidence + 
                             median_height * confidence) / total_confidence
                        )
                        self.confidence_map[gy, gx] = min(total_confidence, 1.0)
    
    def update_occupancy_map(self, points: np.ndarray):
        """Update occupancy map based on point cloud data"""
        if len(points) == 0:
            return
            
        # Reset occupancy for current update
        current_occupancy = np.zeros_like(self.occupancy_map)
        
        for point in points:
            world_x, world_y, world_z = point
            grid_x, grid_y = self.world_to_grid(world_x, world_y)
            
            # Check if within grid bounds
            if (0 <= grid_x < self.config.width and 
                0 <= grid_y < self.config.height):
                
                # Check if point represents an obstacle
                ground_height = self.height_map[grid_y, grid_x]
                if not np.isnan(ground_height):
                    height_diff = world_z - ground_height
                    
                    # Mark as obstacle if significantly above ground
                    if height_diff > self.config.robot_height:
                        current_occupancy[grid_y, grid_x] = 1
        
        # Update occupancy map with temporal filtering
        self.occupancy_map = np.maximum(self.occupancy_map * 0.9, current_occupancy).astype(np.uint8)
    
    def compute_traversability(self):
        """Compute traversability map based on slope analysis"""
        if np.all(np.isnan(self.height_map)):
            return
            
        # Fill NaN values with interpolation for gradient computation
        height_filled = self.height_map.copy()
        mask = ~np.isnan(height_filled)
        
        if np.sum(mask) < 4:  # Need at least 4 points for interpolation
            return
            
        # Interpolate missing values
        coords = np.array(np.nonzero(mask)).T
        values = height_filled[mask]
        
        grid_y, grid_x = np.mgrid[0:self.config.height, 0:self.config.width]
        height_filled = griddata(coords, values, (grid_y, grid_x), method='linear', fill_value=0)
        
        # Compute gradients
        grad_y, grad_x = np.gradient(height_filled, self.config.resolution)
        slope = np.sqrt(grad_x**2 + grad_y**2)
        
        # Update traversability map
        self.traversability_map = np.zeros_like(self.traversability_map)
        
        # Traversable if slope is acceptable and not occupied
        traversable_mask = (slope <= self.config.traversability_threshold) & (self.occupancy_map == 0)
        self.traversability_map[traversable_mask] = 1
        
        # Apply confidence-based filtering
        low_confidence_mask = self.confidence_map < 0.3
        self.traversability_map[low_confidence_mask] = 0
    
    def detect_dynamic_obstacles(self, sensor_data: SensorData, 
                                previous_occupancy: Optional[np.ndarray] = None) -> List[Tuple[int, int]]:
        """
        Detect dynamic obstacles by comparing current and previous occupancy
        
        Args:
            sensor_data: Current sensor data
            previous_occupancy: Previous occupancy map
            
        Returns:
            List of grid coordinates with dynamic obstacles
        """
        if previous_occupancy is None:
            return []
            
        # Process current sensor data
        self.process_sensor_data(sensor_data)
        
        # Find differences between maps
        diff = self.occupancy_map.astype(int) - previous_occupancy.astype(int)
        
        # Dynamic obstacles are new occupied cells
        dynamic_obstacle_coords = []
        new_obstacles = np.where(diff > 0)
        
        for y, x in zip(new_obstacles[0], new_obstacles[1]):
            dynamic_obstacle_coords.append((x, y))  # Note: returning as (x, y)
            
        return dynamic_obstacle_coords
    
    def process_sensor_data(self, sensor_data: SensorData):
        """Process complete sensor data update"""
        # Convert depth to point cloud
        points = self.depth_to_pointcloud(sensor_data.depth_image, sensor_data.camera_pose)
        
        # Update maps
        self.update_height_map(points)
        self.update_occupancy_map(points)
        self.compute_traversability()
    
    def get_maps_for_pathfinding(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get height and obstacle maps formatted for A* pathfinding
        
        Returns:
            Tuple of (height_map, obstacle_map) ready for AStarPlanner
        """
        # Fill NaN values in height map
        height_map_filled = self.height_map.copy()
        nan_mask = np.isnan(height_map_filled)
        height_map_filled[nan_mask] = 0.0  # Set unknown areas to ground level
        
        # Create obstacle map (1 = obstacle, 0 = free)
        obstacle_map = (self.traversability_map == 0).astype(np.uint8)
        
        return height_map_filled, obstacle_map
    
    def save_maps(self, filename: str):
        """Save maps to file"""
        maps_data = {
            'height_map': self.height_map.tolist(),
            'occupancy_map': self.occupancy_map.tolist(),
            'traversability_map': self.traversability_map.tolist(),
            'confidence_map': self.confidence_map.tolist(),
            'config': {
                'resolution': self.config.resolution,
                'width': self.config.width,
                'height': self.config.height,
                'world_origin': self.world_origin
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(maps_data, f, indent=2)
    
    def load_maps(self, filename: str):
        """Load maps from file"""
        with open(filename, 'r') as f:
            maps_data = json.load(f)
        
        self.height_map = np.array(maps_data['height_map'])
        self.occupancy_map = np.array(maps_data['occupancy_map'], dtype=np.uint8)
        self.traversability_map = np.array(maps_data['traversability_map'], dtype=np.uint8)
        self.confidence_map = np.array(maps_data['confidence_map'], dtype=np.float32)

class WebotsIntegration:
    """
    Integration layer for Webots robot controller
    """
    
    def __init__(self, mapper: EnvironmentMapper):
        self.mapper = mapper
        self.robot = None  # Will be set by Webots controller
        self.camera = None
        self.range_finder = None
        self.gps = None
        self.compass = None
        
    def initialize_webots_devices(self, robot):
        """Initialize Webots device references"""
        self.robot = robot
        
        # Get device references (these would be called in actual Webots controller)
        # self.camera = robot.getDevice('camera')
        # self.range_finder = robot.getDevice('range_finder')
        # self.gps = robot.getDevice('gps')
        # self.compass = robot.getDevice('compass')
        
        # Enable devices
        # timestep = int(robot.getBasicTimeStep())
        # self.camera.enable(timestep)
        # self.range_finder.enable(timestep)
        # self.gps.enable(timestep)
        # self.compass.enable(timestep)
    
    def get_robot_pose(self) -> Tuple[float, float, float, float, float, float]:
        """Get current robot pose from Webots sensors"""
        # This would be implemented with actual Webots API calls
        # position = self.gps.getValues()
        # orientation = self.compass.getValues()
        
        # For now, return dummy values
        return (0.0, 0.0, 0.3, 0.0, 0.0, 0.0)  # x, y, z, roll, pitch, yaw
    
    def get_sensor_data(self) -> SensorData:
        """Get current sensor data from Webots"""
        # This would get actual sensor data from Webots
        # rgb_image = np.array(self.camera.getImage()).reshape((height, width, 4))[:,:,:3]
        # depth_image = np.array(self.range_finder.getRangeImage()).reshape((height, width))
        
        # For now, return dummy data
        rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth_image = np.ones((480, 640), dtype=np.float32) * 2.0
        
        camera_pose = self.get_robot_pose()
        timestamp = 0.0  # Would use robot.getTime()
        
        return SensorData(rgb_image, depth_image, camera_pose, timestamp)

# Example usage and testing
def test_environment_mapping():
    """Test the environment mapping system"""
    config = GridMapConfig(resolution=0.1, width=200, height=200)
    mapper = EnvironmentMapper(config, world_origin=(-10.0, -10.0))
    
    # Simulate sensor data
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Create a simple depth image with some obstacles
    depth_image = np.ones((480, 640), dtype=np.float32) * 3.0
    depth_image[200:280, 300:340] = 1.0  # Close obstacle
    depth_image[100:150, 500:600] = 2.0  # Medium distance obstacle
    
    camera_pose = (0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    sensor_data = SensorData(rgb_image, depth_image, camera_pose, 0.0)
    
    # Process sensor data
    mapper.process_sensor_data(sensor_data)
    
    # Get maps for pathfinding
    height_map, obstacle_map = mapper.get_maps_for_pathfinding()
    
    print(f"Height map shape: {height_map.shape}")
    print(f"Obstacle map shape: {obstacle_map.shape}")
    print(f"Number of obstacles: {np.sum(obstacle_map)}")
    print(f"Height range: {np.nanmin(height_map):.2f} to {np.nanmax(height_map):.2f}")
    
    return mapper

if __name__ == "__main__":
    test_mapper = test_environment_mapping()
    print("Environment mapping test completed successfully!")
