#!/usr/bin/env python3

"""
Boundary Supervisor Controller for Construction Site

This controller monitors all moving objects (robots and humans) in the simulation
and prevents them from leaving the defined boundaries of the construction site.
It uses the Supervisor API to track positions and apply corrective forces when needed.

Author: AI Assistant
"""

from controller import Supervisor
import math

class BoundarySupervisor:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        # Define construction site boundaries (50x50 arena with 0.5m walls)
        self.boundary_limits = {
            'x_min': -24.5,  # Leave some margin from the wall
            'x_max': 24.5,
            'y_min': -24.5,
            'y_max': 24.5,
            'z_min': 0.0,    # Ground level
            'z_max': 20.0    # Maximum height
        }
        
        # Objects to monitor (will be populated dynamically)
        self.monitored_objects = {}
        
        # Warning and correction thresholds
        self.warning_distance = 2.0  # Distance from boundary to start warning
        self.correction_force = 50.0  # Force magnitude for boundary correction
        
        print("Boundary Supervisor initialized")
        print(f"Monitoring area: X[{self.boundary_limits['x_min']:.1f}, {self.boundary_limits['x_max']:.1f}], "
              f"Y[{self.boundary_limits['y_min']:.1f}, {self.boundary_limits['y_max']:.1f}]")
    
    def discover_objects(self):
        """Discover and register all movable objects in the simulation"""
        # Get all nodes in the simulation
        root_node = self.supervisor.getRoot()
        children_field = root_node.getField('children')
        
        for i in range(children_field.getCount()):
            node = children_field.getMFNode(i)
            if node is not None:
                node_name = node.getDef()
                if not node_name:
                    node_name = node.getTypeName()
                
                # Check if this is a movable object we should monitor
                if self.should_monitor_object(node, node_name):
                    self.monitored_objects[node_name] = {
                        'node': node,
                        'translation_field': node.getField('translation'),
                        'last_position': [0, 0, 0],
                        'warning_count': 0
                    }
                    print(f"Monitoring object: {node_name}")
    
    def should_monitor_object(self, node, node_name):
        """Determine if an object should be monitored for boundary violations"""
        node_type = node.getTypeName()
        
        # Monitor robots and pedestrians
        if node_type in ['Robot', 'Go2', 'Pedestrian']:
            return True
        
        # Monitor objects with specific names
        monitored_names = ['go2_robot', 'worker_1', 'worker_2', 'worker_3']
        if node_name in monitored_names:
            return True
        
        # Check if object has physics (movable)
        physics_field = node.getField('physics')
        if physics_field and physics_field.getSFNode() is not None:
            return True
        
        return False
    
    def get_object_position(self, obj_info):
        """Get current position of an object"""
        if obj_info['translation_field']:
            return obj_info['translation_field'].getSFVec3f()
        return [0, 0, 0]
    
    def is_outside_boundaries(self, position):
        """Check if position is outside the defined boundaries"""
        x, y, z = position
        
        return (x < self.boundary_limits['x_min'] or x > self.boundary_limits['x_max'] or
                y < self.boundary_limits['y_min'] or y > self.boundary_limits['y_max'] or
                z < self.boundary_limits['z_min'] or z > self.boundary_limits['z_max'])
    
    def get_distance_to_boundary(self, position):
        """Calculate minimum distance to any boundary"""
        x, y, z = position
        
        # Distance to each boundary
        dist_x_min = x - self.boundary_limits['x_min']
        dist_x_max = self.boundary_limits['x_max'] - x
        dist_y_min = y - self.boundary_limits['y_min']
        dist_y_max = self.boundary_limits['y_max'] - y
        dist_z_min = z - self.boundary_limits['z_min']
        dist_z_max = self.boundary_limits['z_max'] - z
        
        # Return minimum positive distance
        distances = [d for d in [dist_x_min, dist_x_max, dist_y_min, dist_y_max, dist_z_min, dist_z_max] if d > 0]
        return min(distances) if distances else 0
    
    def correct_position(self, obj_name, obj_info, position):
        """Correct object position if it's outside boundaries"""
        x, y, z = position
        corrected = False
        
        # Correct X coordinate
        if x < self.boundary_limits['x_min']:
            x = self.boundary_limits['x_min'] + 0.5
            corrected = True
        elif x > self.boundary_limits['x_max']:
            x = self.boundary_limits['x_max'] - 0.5
            corrected = True
        
        # Correct Y coordinate
        if y < self.boundary_limits['y_min']:
            y = self.boundary_limits['y_min'] + 0.5
            corrected = True
        elif y > self.boundary_limits['y_max']:
            y = self.boundary_limits['y_max'] - 0.5
            corrected = True
        
        # Correct Z coordinate (keep above ground, below ceiling)
        if z < self.boundary_limits['z_min']:
            z = self.boundary_limits['z_min'] + 0.1
            corrected = True
        elif z > self.boundary_limits['z_max']:
            z = self.boundary_limits['z_max'] - 0.5
            corrected = True
        
        if corrected:
            # Set the corrected position
            if obj_info['translation_field']:
                obj_info['translation_field'].setSFVec3f([x, y, z])
                print(f"Corrected {obj_name} position to [{x:.2f}, {y:.2f}, {z:.2f}]")
                
                # Reset velocity if the object has physics
                node = obj_info['node']
                physics_field = node.getField('physics')
                if physics_field and physics_field.getSFNode() is not None:
                    # Try to reset velocity (if supported)
                    try:
                        node.resetPhysics()
                    except:
                        pass  # Not all objects support physics reset
        
        return corrected
    
    def apply_boundary_force(self, obj_name, obj_info, position):
        """Apply a gentle force to guide objects away from boundaries"""
        x, y, z = position
        force_x = force_y = force_z = 0
        
        # Calculate repulsive forces from boundaries
        if x < self.boundary_limits['x_min'] + self.warning_distance:
            force_x = self.correction_force * (self.boundary_limits['x_min'] + self.warning_distance - x)
        elif x > self.boundary_limits['x_max'] - self.warning_distance:
            force_x = -self.correction_force * (x - (self.boundary_limits['x_max'] - self.warning_distance))
        
        if y < self.boundary_limits['y_min'] + self.warning_distance:
            force_y = self.correction_force * (self.boundary_limits['y_min'] + self.warning_distance - y)
        elif y > self.boundary_limits['y_max'] - self.warning_distance:
            force_y = -self.correction_force * (y - (self.boundary_limits['y_max'] - self.warning_distance))
        
        # Apply force if needed (this is a simplified approach)
        if abs(force_x) > 0.1 or abs(force_y) > 0.1:
            obj_info['warning_count'] += 1
            if obj_info['warning_count'] % 100 == 0:  # Print warning every 100 steps
                print(f"Warning: {obj_name} approaching boundary at [{x:.2f}, {y:.2f}, {z:.2f}]")
    
    def monitor_objects(self):
        """Monitor all registered objects for boundary violations"""
        for obj_name, obj_info in self.monitored_objects.items():
            try:
                # Get current position
                current_pos = self.get_object_position(obj_info)
                
                # Check if object is outside boundaries
                if self.is_outside_boundaries(current_pos):
                    print(f"Boundary violation detected for {obj_name} at {current_pos}")
                    self.correct_position(obj_name, obj_info, current_pos)
                else:
                    # Check if object is approaching boundaries
                    distance_to_boundary = self.get_distance_to_boundary(current_pos)
                    if distance_to_boundary < self.warning_distance:
                        self.apply_boundary_force(obj_name, obj_info, current_pos)
                
                # Update last position
                obj_info['last_position'] = current_pos
                
            except Exception as e:
                print(f"Error monitoring {obj_name}: {e}")
    
    def run(self):
        """Main control loop"""
        # Discover objects to monitor
        self.discover_objects()
        
        # Main simulation loop
        step_count = 0
        while self.supervisor.step(self.timestep) != -1:
            step_count += 1
            
            # Monitor objects every step
            self.monitor_objects()
            
            # Rediscover objects periodically (in case new objects are added)
            if step_count % 1000 == 0:  # Every ~16 seconds at 16ms timestep
                self.discover_objects()

if __name__ == "__main__":
    supervisor = BoundarySupervisor()
    supervisor.run() 