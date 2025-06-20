#!/usr/bin/env python3
"""
A* Pathfinding Algorithm for Unitree Go2 Robot in 2.5D Construction Environment
Supports dynamic obstacle detection, terrain traversability, and real-time pathfinding
"""

import numpy as np
import heapq
import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from enum import Enum

class TerrainType(Enum):
    TRAVERSABLE = 0
    OBSTACLE = 1
    STEEP_SLOPE = 2
    UNSAFE_HEIGHT = 3
    DYNAMIC_OBSTACLE = 4

@dataclass
class Node:
    """Represents a node in the A* search"""
    x: int
    y: int
    z: float  # Height for 2.5D
    g_cost: float = float('inf')
    h_cost: float = 0.0
    f_cost: float = float('inf')
    parent: Optional['Node'] = None
    terrain_type: TerrainType = TerrainType.TRAVERSABLE
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class AStarPlanner:
    """
    A* pathfinding implementation for Unitree Go2 robot navigation
    in 2.5D construction environments
    """
    
    def __init__(self, grid_resolution: float = 0.1, max_slope: float = 0.3, 
                 max_step_height: float = 0.15, robot_radius: float = 0.3):
        """
        Initialize A* planner with robot constraints
        
        Args:
            grid_resolution: Size of each grid cell in meters
            max_slope: Maximum traversable slope (rise/run)
            max_step_height: Maximum step height robot can climb (meters)
            robot_radius: Robot radius for collision checking (meters)
        """
        self.grid_resolution = grid_resolution
        self.max_slope = max_slope
        self.max_step_height = max_step_height
        self.robot_radius = robot_radius
        
        # 8-directional movement (including diagonals)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        # Direction costs (diagonal movement costs more)
        self.direction_costs = [
            math.sqrt(2), 1, math.sqrt(2),
            1,               1,
            math.sqrt(2), 1, math.sqrt(2)
        ]
        
        self.height_map = None
        self.obstacle_map = None
        self.dynamic_obstacles = set()
        
    def set_environment(self, height_map: np.ndarray, obstacle_map: np.ndarray):
        """Set the environment for pathfinding."""
        self.height_map = height_map
        self.obstacle_map = self.inflate_obstacles(obstacle_map, self.robot_radius)
        self.grid_height, self.grid_width = self.height_map.shape
        
    def inflate_obstacles(self, obstacle_map: np.ndarray, radius: float) -> np.ndarray:
        """Inflate obstacles to account for robot radius, without scipy."""
        inflation_radius_grid = int(np.ceil(radius / self.grid_resolution))
        if inflation_radius_grid == 0:
            return obstacle_map.copy()

        inflated_map = obstacle_map.copy()
        h, w = obstacle_map.shape
        
        # Get coordinates of all obstacles
        obstacle_y, obstacle_x = np.where(obstacle_map)

        # Create a grid of coordinates for the inflation kernel
        y_offsets, x_offsets = np.mgrid[-inflation_radius_grid:inflation_radius_grid + 1,
                                        -inflation_radius_grid:inflation_radius_grid + 1]
        
        # Filter for a circular kernel
        kernel_mask = y_offsets**2 + x_offsets**2 <= inflation_radius_grid**2
        y_offsets = y_offsets[kernel_mask]
        x_offsets = x_offsets[kernel_mask]
        
        # Expand obstacle coordinates with the kernel
        # This creates arrays of all coordinates to be marked as obstacles
        all_y = obstacle_y[:, np.newaxis] + y_offsets
        all_x = obstacle_x[:, np.newaxis] + x_offsets
        
        # Clip coordinates to be within map bounds
        valid_y = np.clip(all_y, 0, h - 1)
        valid_x = np.clip(all_x, 0, w - 1)
        
        # Mark inflated obstacles on the map
        inflated_map[valid_y, valid_x] = True
        
        return inflated_map
    
    def add_dynamic_obstacle(self, x: int, y: int, radius: int = 2):
        """Add dynamic obstacle detected by sensors"""
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    self.dynamic_obstacles.add((x + dx, y + dy))
    
    def clear_dynamic_obstacles(self):
        """Clear all dynamic obstacles"""
        self.dynamic_obstacles.clear()
    
    def is_valid_position(self, x: int, y: int) -> bool:
        """Check if position is within bounds"""
        return 0 <= x < self.grid_height and 0 <= y < self.grid_width
    
    def get_terrain_type(self, x: int, y: int, prev_x: int = None, prev_y: int = None) -> TerrainType:
        """
        Determine terrain type and traversability
        
        Args:
            x, y: Current position
            prev_x, prev_y: Previous position for slope calculation
        """
        if not self.is_valid_position(x, y):
            return TerrainType.OBSTACLE
            
        # Check for static obstacles
        if self.obstacle_map[x, y] == 1:
            return TerrainType.OBSTACLE
            
        # Check for dynamic obstacles
        if (x, y) in self.dynamic_obstacles:
            return TerrainType.DYNAMIC_OBSTACLE
            
        # Check step height if previous position given
        if prev_x is not None and prev_y is not None:
            height_diff = abs(self.height_map[x, y] - self.height_map[prev_x, prev_y])
            if height_diff > self.max_step_height:
                return TerrainType.UNSAFE_HEIGHT
                
        # Check slope constraint
        if prev_x is not None and prev_y is not None:
            distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2) * self.grid_resolution
            if distance > 0:
                slope = abs(self.height_map[x, y] - self.height_map[prev_x, prev_y]) / distance
                if slope > self.max_slope:
                    return TerrainType.STEEP_SLOPE
                    
        return TerrainType.TRAVERSABLE
    
    def heuristic(self, node: Node, goal: Node) -> float:
        """
        Calculate heuristic cost (3D Euclidean distance with height penalty)
        """
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        dz = abs(node.z - goal.z)
        
        # 3D distance with slight height penalty
        return math.sqrt(dx*dx + dy*dy + dz*dz*0.1)
    
    def get_movement_cost(self, current: Node, neighbor_x: int, neighbor_y: int, 
                         direction_idx: int) -> float:
        """
        Calculate movement cost considering terrain and height changes
        """
        base_cost = self.direction_costs[direction_idx]
        
        # Height change penalty
        height_diff = abs(self.height_map[neighbor_x, neighbor_y] - current.z)
        height_penalty = height_diff * 2.0  # Penalize elevation changes
        
        # Terrain type penalty
        terrain_penalty = 0.0
        terrain_type = self.get_terrain_type(neighbor_x, neighbor_y, current.x, current.y)
        
        if terrain_type == TerrainType.OBSTACLE or terrain_type == TerrainType.DYNAMIC_OBSTACLE:
            return float('inf')
        elif terrain_type == TerrainType.STEEP_SLOPE:
            terrain_penalty = 5.0
        elif terrain_type == TerrainType.UNSAFE_HEIGHT:
            return float('inf')
            
        return base_cost + height_penalty + terrain_penalty
    
    def find_path(self, start_grid: tuple, goal_grid: tuple) -> list:
        """
        Find path from start to goal using A* algorithm
        
        Args:
            start_grid: (x, y) start position in grid coordinates
            goal_grid: (x, y) goal position in grid coordinates
            
        Returns:
            List of (x, y, z) waypoints or None if no path found
        """
        if self.height_map is None or self.obstacle_map is None:
            raise ValueError("Environment maps not set. Call set_environment() first.")
            
        start_x, start_y = start_grid
        goal_x, goal_y = goal_grid
        
        if not self.is_valid_position(start_x, start_y) or not self.is_valid_position(goal_x, goal_y):
            print("Error: Start or goal is outside of map bounds.")
            return None

        # Check if start or goal is in an obstacle AFTER inflation
        if self.get_terrain_type(start_x, start_y) == TerrainType.OBSTACLE:
            print(f"Error: Start position ({start_x}, {start_y}) is on an obstacle.")
            return None
        if self.get_terrain_type(goal_x, goal_y) == TerrainType.OBSTACLE:
            print(f"Error: Goal position ({goal_x}, {goal_y}) is on an obstacle.")
            return None

        # Create start and goal nodes
        start_node = Node(start_x, start_y, height=self.height_map[start_y, start_x])
        goal_node = Node(goal_x, goal_y, height=self.height_map[goal_y, goal_x])
        start_node.h_cost = self.heuristic(start_node, goal_node)
        start_node.f_cost = start_node.g_cost + start_node.h_cost

        # Priority queue for open set
        open_set = []
        heapq.heappush(open_set, start_node)
        
        # Track visited nodes
        all_nodes = {(start_x, start_y): start_node}
        closed_set = set()
        
        while open_set:
            current = heapq.heappop(open_set)
            
            # Goal reached
            if current.x == goal_x and current.y == goal_y:
                return self.reconstruct_path(current)
                
            closed_set.add((current.x, current.y))
            
            # Explore neighbors
            for i, (dx, dy) in enumerate(self.directions):
                neighbor_x = current.x + dx
                neighbor_y = current.y + dy
                
                if not self.is_valid_position(neighbor_x, neighbor_y):
                    continue
                
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                    
                # Calculate movement cost
                movement_cost = self.get_movement_cost(current, neighbor_x, neighbor_y, i)
                if movement_cost == float('inf'):
                    continue
                    
                tentative_g_cost = current.g_cost + movement_cost
                
                # Create or get neighbor node
                neighbor = all_nodes.get((neighbor_x, neighbor_y))
                if neighbor is None:
                    neighbor_height = self.height_map[neighbor_y, neighbor_x]
                    neighbor = Node(neighbor_x, neighbor_y, height=neighbor_height)
                    all_nodes[(neighbor_x, neighbor_y)] = neighbor
                
                # Update if better path found
                if tentative_g_cost < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    # Add to open set if not already there
                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)
        
        return None  # No path found
    
    def reconstruct_path(self, current: Node) -> list:
        """Reconstruct path from goal to start"""
        path = []
        while current is not None:
            path.append((current.x, current.y, current.z))
            current = current.parent
        return path[::-1]  # Return reversed path
    
    def smooth_path(self, path: List[Tuple[int, int, float]], 
                   smoothing_factor: float = 0.5) -> List[Tuple[int, int, float]]:
        """
        Apply path smoothing to reduce sharp turns
        """
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]  # Keep start point
        
        for i in range(1, len(path) - 1):
            prev_point = np.array(path[i-1])
            curr_point = np.array(path[i])
            next_point = np.array(path[i+1])
            
            # Weighted average for smoothing
            smoothed_point = (1 - smoothing_factor) * curr_point + \
                           smoothing_factor * 0.5 * (prev_point + next_point)
            
            smoothed.append(tuple(smoothed_point))
            
        smoothed.append(path[-1])  # Keep end point
        return smoothed
    
    def grid_to_world(self, grid_path: List[Tuple[int, int, float]], 
                     world_origin: Tuple[float, float]) -> List[Tuple[float, float, float]]:
        """
        Convert grid coordinates to world coordinates
        
        Args:
            grid_path: Path in grid coordinates
            world_origin: (x, y) origin of the grid in world coordinates
            
        Returns:
            Path in world coordinates
        """
        world_path = []
        origin_x, origin_y = world_origin
        
        for grid_x, grid_y, height in grid_path:
            world_x = origin_x + grid_x * self.grid_resolution
            world_y = origin_y + grid_y * self.grid_resolution
            world_path.append((world_x, world_y, height))
            
        return world_path

# Example usage and testing functions
def test_astar():
    """Test the A* planner with a simple scenario"""
    planner = AStarPlanner(grid_resolution=0.1, max_slope=0.3)
    
    # Create simple test environment
    height_map = np.random.uniform(0, 0.5, (50, 50))
    obstacle_map = np.zeros((50, 50))
    
    # Add some obstacles
    obstacle_map[20:30, 20:30] = 1
    
    planner.set_environment(height_map, obstacle_map)
    
    # Find path
    start = (5, 5)
    goal = (45, 45)
    
    path = planner.find_path(start, goal)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        smoothed_path = planner.smooth_path(path)
        world_path = planner.grid_to_world(smoothed_path, (0.0, 0.0))
        return world_path
    else:
        print("No path found")
        return None

if __name__ == "__main__":
    test_path = test_astar()
    if test_path:
        print("Sample waypoints:")
        for i, waypoint in enumerate(test_path[:5]):
            print(f"  {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f})")
