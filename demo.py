#!/usr/bin/env python3
"""
Demonstration of Unitree Go2 Construction Site Navigation System
Shows integration of A* pathfinding with 2.5D environment mapping
"""

import numpy as np
from ConvertTo2_5D import EnvironmentMapper, GridMapConfig, SensorData
import importlib

# Import A* module
from astar_pathfinding import AStarPlanner

def create_construction_site_environment():
    """Create a simulated construction site environment"""
    print("Creating construction site environment...")
    
    # Grid configuration for 50x50 meter area
    config = GridMapConfig(
        resolution=0.2,     # 20cm resolution
        width=250,          # 50m wide
        height=250,         # 50m deep
        max_height=5.0,
        min_height=-1.0,
        robot_height=0.3,
        traversability_threshold=0.3
    )
    
    # Initialize mapper
    mapper = EnvironmentMapper(config, world_origin=(-25.0, -25.0))
    
    # Create synthetic sensor data representing construction site
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Create depth image with construction elements
    depth_image = np.ones((480, 640), dtype=np.float32) * 10.0  # Background
    
    # Add scaffolding (vertical structures)
    depth_image[100:150, 200:250] = 3.0  # Scaffolding 1
    depth_image[300:350, 400:450] = 2.5  # Scaffolding 2
    depth_image[200:250, 500:550] = 4.0  # Crane
    
    # Add material piles (irregular shapes)
    depth_image[350:400, 100:180] = 1.5  # Material pile 1
    depth_image[50:120, 450:520] = 2.0   # Material pile 2
    
    # Add barriers/walls
    depth_image[150:200, 300:310] = 1.8  # Wall segment
    depth_image[400:450, 350:360] = 1.6  # Barrier
    
    # Robot position and orientation
    camera_pose = (0.0, 0.0, 0.5, 0.0, 0.0, 0.0)  # At origin looking forward
    
    sensor_data = SensorData(rgb_image, depth_image, camera_pose, 0.0)
    
    # Process sensor data to create maps
    mapper.process_sensor_data(sensor_data)
    
    return mapper

def demonstrate_pathfinding(mapper):
    """Demonstrate A* pathfinding in the construction site"""
    print("Setting up A* pathfinding...")
    
    # Initialize A* planner
    planner = AStarPlanner(
        grid_resolution=0.2,
        max_slope=0.3,
        max_step_height=0.15,
        robot_radius=0.4
    )
    
    # Get maps for pathfinding
    height_map, obstacle_map = mapper.get_maps_for_pathfinding()
    planner.set_environment(height_map, obstacle_map)
    
    print(f"Environment map size: {height_map.shape}")
    print(f"Obstacles detected: {np.sum(obstacle_map)} cells")
    
    # Define navigation goals
    test_goals = [
        ((-20, -20), (15, 15)),    # Diagonal across site
        ((-15, 10), (10, -15)),    # Around obstacles
        ((0, -20), (0, 20)),       # North-south traverse
        ((-20, 0), (20, 0))        # East-west traverse
    ]
    
    successful_paths = 0
    
    for i, ((start_x, start_y), (goal_x, goal_y)) in enumerate(test_goals):
        print(f"\nTest {i+1}: From ({start_x}, {start_y}) to ({goal_x}, {goal_y})")
        
        # Convert world coordinates to grid coordinates
        start_grid = mapper.world_to_grid(start_x, start_y)
        goal_grid = mapper.world_to_grid(goal_x, goal_y)
        
        # Find path
        path = planner.find_path(start_grid, goal_grid)
        
        if path:
            print(f"  ✓ Path found with {len(path)} waypoints")
            
            # Convert back to world coordinates
            world_path = planner.grid_to_world(path, mapper.world_origin)
            
            # Apply path smoothing
            smoothed_path = planner.smooth_path(path)
            world_path_smoothed = planner.grid_to_world(smoothed_path, mapper.world_origin)
            
            print(f"  ✓ Path smoothed to {len(world_path_smoothed)} waypoints")
            
            # Calculate path length
            path_length = 0
            for j in range(1, len(world_path_smoothed)):
                dx = world_path_smoothed[j][0] - world_path_smoothed[j-1][0]
                dy = world_path_smoothed[j][1] - world_path_smoothed[j-1][1]
                path_length += np.sqrt(dx*dx + dy*dy)
            
            print(f"  ✓ Total path length: {path_length:.2f} meters")
            successful_paths += 1
            
        else:
            print("  ✗ No path found")
    
    print(f"\nPathfinding summary: {successful_paths}/{len(test_goals)} paths found")
    return successful_paths > 0

def demonstrate_dynamic_obstacles(mapper, planner):
    """Demonstrate dynamic obstacle detection and replanning"""
    print("\nDemonstrating dynamic obstacle detection...")
    
    # Store original occupancy map
    original_occupancy = mapper.occupancy_map.copy()
    
    # Create new sensor data with moving worker/equipment
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_image = np.ones((480, 640), dtype=np.float32) * 10.0
    
    # Add original obstacles
    depth_image[100:150, 200:250] = 3.0
    depth_image[300:350, 400:450] = 2.5
    
    # Add new dynamic obstacle (moving worker/equipment)
    depth_image[200:220, 300:320] = 1.5  # Worker in new position
    
    camera_pose = (0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    new_sensor_data = SensorData(rgb_image, depth_image, camera_pose, 1.0)
    
    # Detect dynamic obstacles
    dynamic_obstacles = mapper.detect_dynamic_obstacles(new_sensor_data, original_occupancy)
    
    if dynamic_obstacles:
        print(f"  ✓ Detected {len(dynamic_obstacles)} dynamic obstacles")
        
        # Add to planner
        planner.clear_dynamic_obstacles()
        for obs_x, obs_y in dynamic_obstacles[:5]:  # Limit to first 5 for demo
            planner.add_dynamic_obstacle(obs_x, obs_y, radius=2)
        
        print("  ✓ Updated pathfinding with dynamic obstacles")
        
        # Test replanning
        start_grid = mapper.world_to_grid(-10, -10)
        goal_grid = mapper.world_to_grid(10, 10)
        
        new_path = planner.find_path(start_grid, goal_grid)
        if new_path:
            print(f"  ✓ Replanned path with {len(new_path)} waypoints")
        else:
            print("  ✗ Replanning failed")
            
    else:
        print("  ✗ No dynamic obstacles detected")

def visualize_environment(mapper):
    """Create a simple text-based visualization of the environment"""
    print("\nEnvironment Map Visualization:")
    print("Legend: . = free space, # = obstacle, ? = unknown")
    
    height_map, obstacle_map = mapper.get_maps_for_pathfinding()
    
    # Create downsampled version for display
    step = max(1, obstacle_map.shape[0] // 50)
    display_map = obstacle_map[::step, ::step]
    
    for row in display_map:
        line = ""
        for cell in row:
            if cell == 1:
                line += "#"
            else:
                line += "."
        print(line)

def main():
    """Main demonstration function"""
    print("=" * 60)
    print("Unitree Go2 Construction Site Navigation Demonstration")
    print("=" * 60)
    
    try:
        # Create construction site environment
        mapper = create_construction_site_environment()
        print("✓ Environment mapping completed")
        
        # Demonstrate pathfinding
        pathfinding_success = demonstrate_pathfinding(mapper)
        
        if pathfinding_success:
            print("✓ Pathfinding demonstration completed")
            
            # Initialize planner for dynamic obstacle demo
            planner = AStarPlanner(grid_resolution=0.2)
            height_map, obstacle_map = mapper.get_maps_for_pathfinding()
            planner.set_environment(height_map, obstacle_map)
            
            # Demonstrate dynamic obstacles
            demonstrate_dynamic_obstacles(mapper, planner)
            
            # Show environment visualization
            visualize_environment(mapper)
            
        else:
            print("⚠ Pathfinding demonstration had issues")
        
        print("\n" + "=" * 60)
        print("Demonstration completed successfully!")
        print("The system is ready for integration with Webots.")
        print("=" * 60)
        
    except Exception as e:
        print(f"Error during demonstration: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 