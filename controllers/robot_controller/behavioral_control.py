#!/usr/bin/env python3
"""
Behavioral Parameter Control System for Unitree Go2 Robot
Implements modular behavioral control with real-time parameter adjustment
"""

import numpy as np
import time
import json
import threading
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum
import random
import math

class BehaviorState(Enum):
    """Robot behavioral states"""
    PATROL = "patrol"
    APPROACH = "approach"
    OBSERVE = "observe"
    RETREAT = "retreat"
    IDLE = "idle"
    EMERGENCY_STOP = "emergency_stop"

class WorkerProximityLevel(Enum):
    """Worker proximity classifications"""
    FAR = "far"          # > 5m
    MEDIUM = "medium"    # 2-5m
    CLOSE = "close"      # 1-2m
    VERY_CLOSE = "very_close"  # < 1m

@dataclass
class BehaviorParameters:
    """Configuration parameters for robot behavior"""
    # Movement speeds (m/s)
    patrol_speed: float = 1.0
    approach_speed: float = 0.5
    observe_speed: float = 0.2
    retreat_speed: float = 0.8
    
    # Proximity thresholds (meters)
    far_threshold: float = 5.0
    medium_threshold: float = 2.0
    close_threshold: float = 1.0
    very_close_threshold: float = 0.5
    
    # Behavioral timing (seconds)
    patrol_duration_min: float = 10.0
    patrol_duration_max: float = 30.0
    observe_duration_min: float = 5.0
    observe_duration_max: float = 15.0
    approach_timeout: float = 20.0
    retreat_timeout: float = 10.0
    
    # Safety constraints
    min_worker_distance: float = 0.5
    max_approach_attempts: int = 3
    emergency_stop_distance: float = 0.3
    
    # Surveillance patterns
    patrol_randomization: float = 0.3  # 0-1, amount of randomness
    surveillance_intensity: float = 0.7  # 0-1, how actively robot seeks workers
    
    # Behavior weights for decision making
    curiosity_weight: float = 0.6
    caution_weight: float = 0.4
    
    def validate(self) -> bool:
        """Validate parameter ranges"""
        try:
            assert 0.1 <= self.patrol_speed <= 3.0
            assert 0.1 <= self.approach_speed <= 2.0
            assert 0.05 <= self.observe_speed <= 1.0
            assert 0.1 <= self.retreat_speed <= 3.0
            
            assert self.emergency_stop_distance < self.very_close_threshold
            assert self.very_close_threshold < self.close_threshold
            assert self.close_threshold < self.medium_threshold
            assert self.medium_threshold < self.far_threshold
            
            assert 0.0 <= self.patrol_randomization <= 1.0
            assert 0.0 <= self.surveillance_intensity <= 1.0
            
            return True
        except AssertionError:
            return False

@dataclass
class WorkerInfo:
    """Information about detected workers"""
    worker_id: int
    position: Tuple[float, float, float]  # x, y, z
    distance: float
    last_seen: float  # timestamp
    proximity_level: WorkerProximityLevel
    attention_score: float = 0.0  # 0-1, how much attention robot should pay

class BehaviorStateMachine:
    """
    Finite state machine for robot behavioral control
    """
    
    def __init__(self, parameters: BehaviorParameters):
        self.parameters = parameters
        self.current_state = BehaviorState.IDLE
        self.previous_state = BehaviorState.IDLE
        self.state_start_time = time.time()
        self.state_duration = 0.0
        
        # Worker tracking
        self.detected_workers: Dict[int, WorkerInfo] = {}
        self.target_worker_id: Optional[int] = None
        self.approach_attempts = 0
        
        # State transition callbacks
        self.state_callbacks: Dict[BehaviorState, List[Callable]] = {
            state: [] for state in BehaviorState
        }
        
        # Behavior history for analysis
        self.behavior_history: List[Dict] = []
        
        # Safety flags
        self.emergency_stop_active = False
        self.safety_override = False
        
    def add_state_callback(self, state: BehaviorState, callback: Callable):
        """Add callback function to be called when entering a state"""
        self.state_callbacks[state].append(callback)
    
    def transition_to_state(self, new_state: BehaviorState, reason: str = ""):
        """Transition to new behavioral state"""
        if new_state == self.current_state:
            return
            
        # Log transition
        transition_data = {
            'timestamp': time.time(),
            'from_state': self.current_state.value,
            'to_state': new_state.value,
            'duration': time.time() - self.state_start_time,
            'reason': reason,
            'workers_detected': len(self.detected_workers),
            'target_worker': self.target_worker_id
        }
        self.behavior_history.append(transition_data)
        
        print(f"State transition: {self.current_state.value} -> {new_state.value} ({reason})")
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()
        
        # Execute callbacks
        for callback in self.state_callbacks[new_state]:
            try:
                callback(new_state, self.previous_state, reason)
            except Exception as e:
                print(f"Error in state callback: {e}")
    
    def update_worker_detection(self, workers: List[WorkerInfo]):
        """Update detected worker information"""
        current_time = time.time()
        
        # Update existing workers and add new ones
        for worker in workers:
            self.detected_workers[worker.worker_id] = worker
        
        # Remove workers not seen recently (>5 seconds)
        workers_to_remove = []
        for worker_id, worker in self.detected_workers.items():
            if current_time - worker.last_seen > 5.0:
                workers_to_remove.append(worker_id)
        
        for worker_id in workers_to_remove:
            del self.detected_workers[worker_id]
            if self.target_worker_id == worker_id:
                self.target_worker_id = None
    
    def get_closest_worker(self) -> Optional[WorkerInfo]:
        """Get information about closest detected worker"""
        if not self.detected_workers:
            return None
            
        closest_worker = min(self.detected_workers.values(), key=lambda w: w.distance)
        return closest_worker
    
    def evaluate_worker_proximity(self, distance: float) -> WorkerProximityLevel:
        """Classify worker proximity based on distance"""
        if distance <= self.parameters.very_close_threshold:
            return WorkerProximityLevel.VERY_CLOSE
        elif distance <= self.parameters.close_threshold:
            return WorkerProximityLevel.CLOSE
        elif distance <= self.parameters.medium_threshold:
            return WorkerProximityLevel.MEDIUM
        else:
            return WorkerProximityLevel.FAR
    
    def check_safety_conditions(self) -> bool:
        """Check if safety conditions require emergency action"""
        closest_worker = self.get_closest_worker()
        
        if closest_worker and closest_worker.distance < self.parameters.emergency_stop_distance:
            if not self.emergency_stop_active:
                self.emergency_stop_active = True
                self.transition_to_state(BehaviorState.EMERGENCY_STOP, "Worker too close")
            return False
        elif self.emergency_stop_active and (not closest_worker or 
                                           closest_worker.distance > self.parameters.very_close_threshold):
            self.emergency_stop_active = False
            self.transition_to_state(BehaviorState.RETREAT, "Resuming from emergency stop")
        
        return not self.emergency_stop_active
    
    def select_target_worker(self) -> Optional[int]:
        """Select target worker based on behavioral parameters"""
        if not self.detected_workers:
            return None
        
        # Score workers based on distance, attention, and behavioral parameters
        scored_workers = []
        
        for worker in self.detected_workers.values():
            # Distance score (closer = higher, but not too close)
            if worker.distance < self.parameters.close_threshold:
                distance_score = 0.3  # Discourage getting too close
            else:
                distance_score = max(0, 1.0 - worker.distance / self.parameters.far_threshold)
            
            # Attention score
            attention_score = worker.attention_score
            
            # Combine scores
            total_score = (distance_score * self.parameters.curiosity_weight + 
                          attention_score * self.parameters.surveillance_intensity)
            
            scored_workers.append((worker.worker_id, total_score))
        
        if scored_workers:
            # Add randomization
            if random.random() < self.parameters.patrol_randomization:
                return random.choice(scored_workers)[0]
            else:
                return max(scored_workers, key=lambda x: x[1])[0]
        
        return None
    
    def update_state_machine(self) -> BehaviorState:
        """Main state machine update logic"""
        if not self.check_safety_conditions():
            return self.current_state
        
        current_time = time.time()
        state_duration = current_time - self.state_start_time
        closest_worker = self.get_closest_worker()
        
        # State-specific logic
        if self.current_state == BehaviorState.IDLE:
            if self.detected_workers:
                self.transition_to_state(BehaviorState.PATROL, "Workers detected")
            
        elif self.current_state == BehaviorState.PATROL:
            if not self.detected_workers:
                if state_duration > self.parameters.patrol_duration_max:
                    self.transition_to_state(BehaviorState.IDLE, "No workers found")
            elif closest_worker:
                if closest_worker.proximity_level in [WorkerProximityLevel.CLOSE, WorkerProximityLevel.MEDIUM]:
                    self.target_worker_id = closest_worker.worker_id
                    self.transition_to_state(BehaviorState.APPROACH, "Worker in range")
                elif state_duration > self.parameters.patrol_duration_max:
                    # Continue patrolling but maybe change target
                    self.target_worker_id = self.select_target_worker()
                    if self.target_worker_id:
                        self.transition_to_state(BehaviorState.APPROACH, "Selected new target")
        
        elif self.current_state == BehaviorState.APPROACH:
            if not closest_worker or self.target_worker_id not in self.detected_workers:
                self.transition_to_state(BehaviorState.PATROL, "Lost target worker")
            elif closest_worker.proximity_level == WorkerProximityLevel.CLOSE:
                self.transition_to_state(BehaviorState.OBSERVE, "Reached observation distance")
            elif closest_worker.proximity_level == WorkerProximityLevel.VERY_CLOSE:
                self.transition_to_state(BehaviorState.RETREAT, "Too close to worker")
            elif state_duration > self.parameters.approach_timeout:
                self.approach_attempts += 1
                if self.approach_attempts >= self.parameters.max_approach_attempts:
                    self.transition_to_state(BehaviorState.PATROL, "Max approach attempts reached")
                else:
                    self.transition_to_state(BehaviorState.RETREAT, "Approach timeout")
        
        elif self.current_state == BehaviorState.OBSERVE:
            if not closest_worker or self.target_worker_id not in self.detected_workers:
                self.transition_to_state(BehaviorState.PATROL, "Lost observation target")
            elif closest_worker.proximity_level == WorkerProximityLevel.VERY_CLOSE:
                self.transition_to_state(BehaviorState.RETREAT, "Worker moved too close")
            elif closest_worker.proximity_level == WorkerProximityLevel.FAR:
                self.transition_to_state(BehaviorState.APPROACH, "Worker moved away")
            elif state_duration > self.parameters.observe_duration_max:
                self.transition_to_state(BehaviorState.PATROL, "Observation complete")
        
        elif self.current_state == BehaviorState.RETREAT:
            if not closest_worker:
                self.transition_to_state(BehaviorState.PATROL, "Area clear")
            elif closest_worker.proximity_level in [WorkerProximityLevel.MEDIUM, WorkerProximityLevel.FAR]:
                self.transition_to_state(BehaviorState.PATROL, "Safe distance achieved")
            elif state_duration > self.parameters.retreat_timeout:
                self.transition_to_state(BehaviorState.PATROL, "Retreat timeout")
        
        elif self.current_state == BehaviorState.EMERGENCY_STOP:
            # Handled in check_safety_conditions
            pass
        
        return self.current_state
    
    def get_current_speed(self) -> float:
        """Get speed setting for current state"""
        speed_map = {
            BehaviorState.PATROL: self.parameters.patrol_speed,
            BehaviorState.APPROACH: self.parameters.approach_speed,
            BehaviorState.OBSERVE: self.parameters.observe_speed,
            BehaviorState.RETREAT: self.parameters.retreat_speed,
            BehaviorState.IDLE: 0.0,
            BehaviorState.EMERGENCY_STOP: 0.0
        }
        return speed_map.get(self.current_state, 0.0)
    
    def get_behavior_data(self) -> Dict:
        """Get current behavior state data for logging"""
        return {
            'timestamp': time.time(),
            'current_state': self.current_state.value,
            'state_duration': time.time() - self.state_start_time,
            'target_worker_id': self.target_worker_id,
            'workers_detected': len(self.detected_workers),
            'closest_worker_distance': self.get_closest_worker().distance if self.get_closest_worker() else None,
            'current_speed': self.get_current_speed(),
            'emergency_stop_active': self.emergency_stop_active,
            'approach_attempts': self.approach_attempts
        }

class BehaviorController:
    """
    Main controller class that integrates behavior state machine with robot control
    """
    
    def __init__(self, initial_parameters: BehaviorParameters):
        self.parameters = initial_parameters
        self.state_machine = BehaviorStateMachine(self.parameters)
        
        # Parameter update lock for thread safety
        self.parameter_lock = threading.Lock()
        
        # Behavior logging
        self.behavior_log: List[Dict] = []
        self.logging_enabled = True
        
        # Patrol pattern state
        self.patrol_waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0
        self.last_waypoint_time = time.time()
        
        # Initialize callbacks
        self._setup_state_callbacks()
        
        print("Behavior Controller initialized")
    
    def _setup_state_callbacks(self):
        """Set up callbacks for state transitions"""
        self.state_machine.add_state_callback(BehaviorState.PATROL, self._on_patrol_start)
        self.state_machine.add_state_callback(BehaviorState.APPROACH, self._on_approach_start)
        self.state_machine.add_state_callback(BehaviorState.OBSERVE, self._on_observe_start)
        self.state_machine.add_state_callback(BehaviorState.RETREAT, self._on_retreat_start)
        self.state_machine.add_state_callback(BehaviorState.EMERGENCY_STOP, self._on_emergency_stop)
    
    def _on_patrol_start(self, new_state, previous_state, reason):
        """Callback for entering patrol state"""
        if not self.patrol_waypoints:
            self.generate_patrol_pattern()
        print(f"Starting patrol behavior: {reason}")
    
    def _on_approach_start(self, new_state, previous_state, reason):
        """Callback for entering approach state"""
        print(f"Approaching worker {self.state_machine.target_worker_id}: {reason}")
    
    def _on_observe_start(self, new_state, previous_state, reason):
        """Callback for entering observe state"""
        print(f"Observing worker {self.state_machine.target_worker_id}: {reason}")
    
    def _on_retreat_start(self, new_state, previous_state, reason):
        """Callback for entering retreat state"""
        print(f"Retreating from worker: {reason}")
    
    def _on_emergency_stop(self, new_state, previous_state, reason):
        """Callback for emergency stop"""
        print(f"EMERGENCY STOP ACTIVATED: {reason}")
    
    def update_parameters(self, new_parameters: BehaviorParameters) -> bool:
        """Safely update behavior parameters during runtime"""
        if not new_parameters.validate():
            print("Invalid parameters provided")
            return False
        
        with self.parameter_lock:
            self.parameters = new_parameters
            self.state_machine.parameters = new_parameters
            print("Behavior parameters updated")
            return True
    
    def get_parameters(self) -> BehaviorParameters:
        """Get current parameters (thread-safe)"""
        with self.parameter_lock:
            return self.parameters
    
    def update_worker_detections(self, worker_positions: List[Tuple[int, float, float, float]]):
        """
        Update worker detection data
        
        Args:
            worker_positions: List of (worker_id, x, y, z) tuples
        """
        current_time = time.time()
        workers = []
        
        for worker_id, x, y, z in worker_positions:
            # Calculate distance (assuming robot at origin for simplicity)
            distance = math.sqrt(x*x + y*y)
            proximity_level = self.state_machine.evaluate_worker_proximity(distance)
            
            worker_info = WorkerInfo(
                worker_id=worker_id,
                position=(x, y, z),
                distance=distance,
                last_seen=current_time,
                proximity_level=proximity_level,
                attention_score=random.uniform(0.3, 1.0)  # Would be calculated from actual attention metrics
            )
            workers.append(worker_info)
        
        self.state_machine.update_worker_detection(workers)
    
    def generate_patrol_pattern(self, area_size: float = 20.0):
        """Generate randomized patrol waypoints"""
        num_waypoints = random.randint(4, 8)
        self.patrol_waypoints = []
        
        for _ in range(num_waypoints):
            # Generate points in a rough grid with randomization
            x = random.uniform(-area_size/2, area_size/2)
            y = random.uniform(-area_size/2, area_size/2)
            
            # Add some structure to avoid purely random movement
            if self.parameters.patrol_randomization < 0.5:
                x = round(x / 5.0) * 5.0  # Snap to 5m grid
                y = round(y / 5.0) * 5.0
            
            self.patrol_waypoints.append((x, y))
        
        self.current_waypoint_index = 0
        print(f"Generated patrol pattern with {len(self.patrol_waypoints)} waypoints")
    
    def get_current_target_position(self) -> Optional[Tuple[float, float]]:
        """Get current navigation target based on behavior state"""
        if self.state_machine.current_state == BehaviorState.PATROL:
            if self.patrol_waypoints:
                return self.patrol_waypoints[self.current_waypoint_index]
        
        elif self.state_machine.current_state == BehaviorState.APPROACH:
            if self.state_machine.target_worker_id in self.state_machine.detected_workers:
                worker = self.state_machine.detected_workers[self.state_machine.target_worker_id]
                # Target position slightly away from worker
                return (worker.position[0], worker.position[1])
        
        elif self.state_machine.current_state == BehaviorState.OBSERVE:
            if self.state_machine.target_worker_id in self.state_machine.detected_workers:
                worker = self.state_machine.detected_workers[self.state_machine.target_worker_id]
                return (worker.position[0], worker.position[1])
        
        elif self.state_machine.current_state == BehaviorState.RETREAT:
            closest_worker = self.state_machine.get_closest_worker()
            if closest_worker:
                # Move away from worker
                worker_x, worker_y = closest_worker.position[0], closest_worker.position[1]
                retreat_distance = self.parameters.medium_threshold + 1.0
                angle = math.atan2(-worker_y, -worker_x)  # Opposite direction
                retreat_x = worker_x + retreat_distance * math.cos(angle)
                retreat_y = worker_y + retreat_distance * math.sin(angle)
                return (retreat_x, retreat_y)
        
        return None
    
    def update(self) -> Dict:
        """Main update loop - call this regularly"""
        # Update state machine
        current_state = self.state_machine.update_state_machine()
        
        # Log behavior data
        if self.logging_enabled:
            behavior_data = self.state_machine.get_behavior_data()
            behavior_data['target_position'] = self.get_current_target_position()
            self.behavior_log.append(behavior_data)
        
        # Update patrol waypoint if needed
        if (current_state == BehaviorState.PATROL and self.patrol_waypoints and
            time.time() - self.last_waypoint_time > 10.0):  # 10 seconds per waypoint
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_waypoints)
            self.last_waypoint_time = time.time()
        
        return {
            'current_state': current_state.value,
            'target_position': self.get_current_target_position(),
            'speed': self.state_machine.get_current_speed(),
            'emergency_stop': self.state_machine.emergency_stop_active
        }
    
    def save_behavior_log(self, filename: str):
        """Save behavior log to file"""
        log_data = {
            'parameters': asdict(self.parameters),
            'behavior_history': self.state_machine.behavior_history,
            'behavior_log': self.behavior_log
        }
        
        with open(filename, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"Behavior log saved to {filename}")

# Parameter adjustment interface
class ParameterInterface:
    """Interface for real-time parameter adjustment"""
    
    def __init__(self, controller: BehaviorController):
        self.controller = controller
    
    def set_speed_parameters(self, patrol: float = None, approach: float = None, 
                           observe: float = None, retreat: float = None):
        """Adjust movement speed parameters"""
        params = self.controller.get_parameters()
        
        if patrol is not None:
            params.patrol_speed = max(0.1, min(3.0, patrol))
        if approach is not None:
            params.approach_speed = max(0.1, min(2.0, approach))
        if observe is not None:
            params.observe_speed = max(0.05, min(1.0, observe))
        if retreat is not None:
            params.retreat_speed = max(0.1, min(3.0, retreat))
        
        return self.controller.update_parameters(params)
    
    def set_proximity_thresholds(self, far: float = None, medium: float = None,
                               close: float = None, very_close: float = None):
        """Adjust proximity threshold parameters"""
        params = self.controller.get_parameters()
        
        if far is not None:
            params.far_threshold = max(2.0, min(10.0, far))
        if medium is not None:
            params.medium_threshold = max(1.0, min(params.far_threshold - 0.5, medium))
        if close is not None:
            params.close_threshold = max(0.5, min(params.medium_threshold - 0.5, close))
        if very_close is not None:
            params.very_close_threshold = max(0.2, min(params.close_threshold - 0.2, very_close))
        
        return self.controller.update_parameters(params)
    
    def set_behavioral_weights(self, curiosity: float = None, caution: float = None,
                              surveillance_intensity: float = None, randomization: float = None):
        """Adjust behavioral weight parameters"""
        params = self.controller.get_parameters()
        
        if curiosity is not None:
            params.curiosity_weight = max(0.0, min(1.0, curiosity))
        if caution is not None:
            params.caution_weight = max(0.0, min(1.0, caution))
        if surveillance_intensity is not None:
            params.surveillance_intensity = max(0.0, min(1.0, surveillance_intensity))
        if randomization is not None:
            params.patrol_randomization = max(0.0, min(1.0, randomization))
        
        return self.controller.update_parameters(params)

# Testing and demonstration
def test_behavioral_control():
    """Test the behavioral control system"""
    print("Testing Behavioral Control System")
    print("=" * 50)
    
    # Initialize system
    params = BehaviorParameters()
    controller = BehaviorController(params)
    interface = ParameterInterface(controller)
    
    # Simulate worker detections
    test_scenarios = [
        # Scenario 1: Single worker at medium distance
        [(1, 3.0, 2.0, 0.0)],
        # Scenario 2: Worker approaches
        [(1, 2.0, 1.5, 0.0)],
        [(1, 1.5, 1.0, 0.0)],
        [(1, 1.0, 0.8, 0.0)],
        # Scenario 3: Worker too close
        [(1, 0.4, 0.3, 0.0)],
        # Scenario 4: Worker moves away
        [(1, 1.5, 1.2, 0.0)],
        [(1, 3.0, 2.5, 0.0)],
        # Scenario 5: No workers
        []
    ]
    
    print("Running simulation scenarios...")
    
    for i, worker_positions in enumerate(test_scenarios):
        print(f"\nScenario {i+1}: Workers at {worker_positions}")
        
        controller.update_worker_detections(worker_positions)
        result = controller.update()
        
        print(f"  State: {result['current_state']}")
        print(f"  Speed: {result['speed']:.2f} m/s")
        print(f"  Target: {result['target_position']}")
        print(f"  Emergency: {result['emergency_stop']}")
        
        time.sleep(0.1)  # Brief pause between scenarios
    
    print("\nTesting parameter adjustment...")
    
    # Test parameter changes
    interface.set_speed_parameters(patrol=0.5, approach=0.3)
    print("✓ Speed parameters adjusted")
    
    interface.set_proximity_thresholds(close=1.5, very_close=0.8)
    print("✓ Proximity thresholds adjusted")
    
    interface.set_behavioral_weights(curiosity=0.8, surveillance_intensity=0.9)
    print("✓ Behavioral weights adjusted")
    
    # Save behavior log
    controller.save_behavior_log("behavior_test_log.json")
    
    print("\n" + "=" * 50)
    print("Behavioral control system test completed!")

if __name__ == "__main__":
    test_behavioral_control() 