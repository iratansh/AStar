#!/usr/bin/env python3
"""
Integrated Behavioral Study System for Unitree Go2 Robot
Combines navigation, behavioral control, data collection, and sensor processing
"""

import time
import threading
import queue
import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
import numpy as np

# Import our custom modules
from behavioral_control import BehaviorController, BehaviorParameters, ParameterInterface
from data_collection import DataCollectionSystem, RobotTelemetry
from sensor_processing import SynchronizedAnalyzer
from astar_pathfinding import AStarPlanner
from ConvertTo2_5D import EnvironmentMapper, GridMapConfig, SensorData

@dataclass
class StudyConfiguration:
    """Configuration for the complete behavioral study"""
    # Session information
    session_id: str
    participant_id: str
    condition: str  # e.g., "baseline", "stress", "fatigue"
    
    # Study parameters
    study_duration: float = 600.0  # 10 minutes default
    data_collection_rate: float = 100.0  # Hz
    sensor_processing_interval: float = 5.0  # seconds
    
    # Robot behavior parameters
    behavior_params: BehaviorParameters = None
    
    # Environment parameters
    environment_size: float = 20.0  # meters
    worker_simulation: bool = True

class IntegratedStudySystem:
    """
    Main integrated system for the behavioral study
    Coordinates all subsystems
    """
    
    def __init__(self, config: StudyConfiguration):
        self.config = config
        self.running = False
        self.start_time = None
        
        # Initialize subsystems
        self._initialize_subsystems()
        
        # Data queues and threading
        self.robot_data_queue = queue.Queue()
        self.sensor_data_queue = queue.Queue()
        self.control_thread = None
        self.processing_thread = None
        
        # Study state
        self.current_robot_state = {}
        self.current_sensor_data = {}
        self.processed_results = []
        
        print(f"Integrated study system initialized for session: {config.session_id}")
    
    def _initialize_subsystems(self):
        """Initialize all subsystems"""
        # Behavioral control system
        behavior_params = self.config.behavior_params or BehaviorParameters()
        self.behavior_controller = BehaviorController(behavior_params)
        self.parameter_interface = ParameterInterface(self.behavior_controller)
        
        # Data collection system
        self.data_collector = DataCollectionSystem(
            self.config.session_id, 
            self.config.participant_id
        )
        
        # Sensor processing system
        self.sensor_analyzer = SynchronizedAnalyzer()
        
        # Navigation system
        map_config = GridMapConfig(
            resolution=0.1,
            width=int(self.config.environment_size * 10),
            height=int(self.config.environment_size * 10),
            max_height=3.0,
            min_height=-1.0
        )
        self.environment_mapper = EnvironmentMapper(
            map_config, 
            world_origin=(-self.config.environment_size/2, -self.config.environment_size/2)
        )
        
        self.path_planner = AStarPlanner(
            grid_resolution=0.1,
            max_slope=0.3,
            max_step_height=0.15,
            robot_radius=0.3
        )
        
        print("All subsystems initialized")
    
    def start_study(self):
        """Start the complete behavioral study"""
        if self.running:
            print("Study already running")
            return
        
        print(f"Starting behavioral study: {self.config.session_id}")
        
        self.running = True
        self.start_time = time.time()
        
        # Start data collection
        self.data_collector.start_collection()
        
        # Log study start
        self.data_collector.log_behavioral_event("study_start", "idle", {
            'condition': self.config.condition,
            'participant_id': self.config.participant_id,
            'expected_duration': self.config.study_duration
        })
        
        # Start control threads
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        print("Study started successfully")
    
    def stop_study(self):
        """Stop the behavioral study"""
        if not self.running:
            return
        
        print("Stopping behavioral study...")
        
        self.running = False
        
        # Wait for threads to finish
        if self.control_thread:
            self.control_thread.join(timeout=5.0)
        if self.processing_thread:
            self.processing_thread.join(timeout=5.0)
        
        # Log study end
        study_duration = time.time() - self.start_time
        self.data_collector.log_behavioral_event("study_end", "idle", {
            'actual_duration': study_duration,
            'planned_duration': self.config.study_duration
        })
        
        # Stop data collection
        self.data_collector.stop_collection()
        
        # Generate final report
        self._generate_study_report()
        
        print(f"Study completed. Duration: {study_duration:.1f} seconds")
    
    def _control_loop(self):
        """Main robot control and data collection loop"""
        last_data_time = time.time()
        data_interval = 1.0 / self.config.data_collection_rate
        
        # Simulated worker positions (in real system, this would come from sensors)
        simulated_workers = self._initialize_simulated_workers()
        
        while self.running:
            current_time = time.time()
            
            # Check study duration
            if current_time - self.start_time > self.config.study_duration:
                print("Study duration reached, stopping...")
                break
            
            # Update simulated worker positions
            if self.config.worker_simulation:
                self._update_simulated_workers(simulated_workers, current_time)
            
            # Update robot behavior system
            worker_positions = [(w['id'], w['x'], w['y'], w['z']) for w in simulated_workers]
            self.behavior_controller.update_worker_detections(worker_positions)
            
            behavior_result = self.behavior_controller.update()
            
            # Update navigation if needed
            target_pos = behavior_result.get('target_position')
            if target_pos:
                self._update_navigation(target_pos)
            
            # Collect robot telemetry
            if current_time - last_data_time >= data_interval:
                self._collect_robot_telemetry(behavior_result, simulated_workers)
                last_data_time = current_time
            
            # Simulate sensor data collection
            self._simulate_sensor_data_collection()
            
            time.sleep(0.01)  # 100Hz control loop
    
    def _processing_loop(self):
        """Sensor data processing loop"""
        last_processing_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Process sensor data at specified intervals
            if current_time - last_processing_time >= self.config.sensor_processing_interval:
                self._process_accumulated_sensor_data()
                last_processing_time = current_time
            
            time.sleep(0.1)  # 10Hz processing loop
    
    def _initialize_simulated_workers(self) -> List[Dict]:
        """Initialize simulated worker positions"""
        workers = []
        num_workers = 3
        
        for i in range(num_workers):
            angle = (2 * np.pi * i) / num_workers
            radius = self.config.environment_size * 0.3
            
            worker = {
                'id': i + 1,
                'x': radius * np.cos(angle),
                'y': radius * np.sin(angle),
                'z': 0.0,
                'velocity': np.random.uniform(0.2, 0.8),  # m/s
                'direction': np.random.uniform(0, 2 * np.pi)
            }
            workers.append(worker)
        
        return workers
    
    def _update_simulated_workers(self, workers: List[Dict], current_time: float):
        """Update simulated worker positions"""
        dt = 0.01  # Time step
        
        for worker in workers:
            # Simple random walk with momentum
            if np.random.random() < 0.1:  # Change direction occasionally
                worker['direction'] += np.random.normal(0, 0.5)
            
            # Update position
            worker['x'] += worker['velocity'] * np.cos(worker['direction']) * dt
            worker['y'] += worker['velocity'] * np.sin(worker['direction']) * dt
            
            # Keep workers within bounds
            max_pos = self.config.environment_size * 0.4
            worker['x'] = np.clip(worker['x'], -max_pos, max_pos)
            worker['y'] = np.clip(worker['y'], -max_pos, max_pos)
            
            # Bounce off boundaries
            if abs(worker['x']) >= max_pos or abs(worker['y']) >= max_pos:
                worker['direction'] += np.pi + np.random.normal(0, 0.3)
    
    def _update_navigation(self, target_position: tuple):
        """Update robot navigation system"""
        # In a real system, this would interface with the actual robot navigation
        # For simulation, we just track the target
        self.current_robot_state['target_position'] = target_position
        self.current_robot_state['navigation_active'] = True
    
    def _collect_robot_telemetry(self, behavior_result: Dict, workers: List[Dict]):
        """Collect and log robot telemetry data"""
        # Simulate robot position (in real system, get from robot sensors)
        robot_pos = self.current_robot_state.get('position', (0.0, 0.0, 0.3))
        
        # Calculate worker distances
        worker_distances = {}
        for worker in workers:
            distance = np.sqrt((worker['x'] - robot_pos[0])**2 + 
                             (worker['y'] - robot_pos[1])**2)
            worker_distances[worker['id']] = distance
        
        # Create telemetry data
        telemetry_data = {
            'position': robot_pos,
            'orientation': (0.0, 0.0, 0.0),  # Simulated
            'velocity': (0.0, 0.0, 0.0),     # Simulated
            'behavioral_state': behavior_result.get('current_state', 'unknown'),
            'target_worker_id': self.behavior_controller.state_machine.target_worker_id,
            'worker_distances': worker_distances,
            'movement_speed': behavior_result.get('speed', 0.0),
            'emergency_status': behavior_result.get('emergency_stop', False)
        }
        
        # Update current state
        self.current_robot_state.update(telemetry_data)
        
        # Send to data collector
        self.data_collector.collect_robot_data(telemetry_data)
        
        # Track worker interactions
        for worker_id, distance in worker_distances.items():
            self.data_collector.track_worker_interaction(
                worker_id, distance, telemetry_data['behavioral_state']
            )
    
    def _simulate_sensor_data_collection(self):
        """Simulate physiological sensor data collection"""
        # In a real system, this would interface with actual sensors
        # For simulation, generate synthetic data
        
        current_time = time.time()
        
        # Simulate stress response based on robot behavior
        behavioral_state = self.current_robot_state.get('behavioral_state', 'idle')
        stress_level = self._calculate_stress_level(behavioral_state)
        
        # Generate synthetic sensor data based on stress level
        sensor_data = {
            'timestamp': current_time,
            'ecg': self._generate_synthetic_ecg(stress_level),
            'eda': self._generate_synthetic_eda(stress_level),
            'eye_tracking': self._generate_synthetic_eye_data(stress_level),
            'fnirs': self._generate_synthetic_fnirs(stress_level)
        }
        
        # Store for processing
        self.current_sensor_data = sensor_data
        self.sensor_data_queue.put(sensor_data)
    
    def _calculate_stress_level(self, behavioral_state: str) -> float:
        """Calculate stress level based on robot behavioral state"""
        stress_map = {
            'idle': 0.1,
            'patrol': 0.2,
            'approach': 0.6,
            'observe': 0.7,
            'retreat': 0.4,
            'emergency_stop': 0.9
        }
        return stress_map.get(behavioral_state, 0.3)
    
    def _generate_synthetic_ecg(self, stress_level: float) -> np.ndarray:
        """Generate synthetic ECG data"""
        # Simple synthetic ECG based on stress level
        duration = 1.0  # 1 second
        fs = 1000
        t = np.linspace(0, duration, int(duration * fs))
        
        # Base heart rate increases with stress
        base_hr = 70 + stress_level * 30  # 70-100 bpm
        
        # Simple ECG simulation
        ecg = np.sin(2 * np.pi * base_hr / 60 * t) + 0.1 * np.random.randn(len(t))
        
        return ecg
    
    def _generate_synthetic_eda(self, stress_level: float) -> np.ndarray:
        """Generate synthetic EDA data"""
        duration = 1.0
        fs = 1000
        t = np.linspace(0, duration, int(duration * fs))
        
        # Base EDA level increases with stress
        base_eda = 5 + stress_level * 3
        
        # Add SCR events based on stress
        eda = np.ones(len(t)) * base_eda + 0.1 * np.random.randn(len(t))
        
        if stress_level > 0.5 and np.random.random() < 0.3:
            # Add SCR peak
            scr_start = int(len(t) * 0.3)
            scr_duration = int(len(t) * 0.2)
            scr_amplitude = stress_level * 2
            scr_decay = np.exp(-np.linspace(0, 3, scr_duration))
            eda[scr_start:scr_start + scr_duration] += scr_amplitude * scr_decay
        
        return eda
    
    def _generate_synthetic_eye_data(self, stress_level: float) -> Dict:
        """Generate synthetic eye tracking data"""
        duration = 1.0
        fs = 1000
        t = np.linspace(0, duration, int(duration * fs))
        
        # Gaze patterns affected by stress
        gaze_variability = 1 + stress_level * 2
        gaze_x = gaze_variability * np.sin(0.5 * t) + 0.5 * np.random.randn(len(t))
        gaze_y = gaze_variability * np.cos(0.3 * t) + 0.3 * np.random.randn(len(t))
        
        # Pupil diameter increases with stress/arousal
        base_pupil = 3.5 + stress_level * 1.0
        pupil_diameter = base_pupil + 0.1 * np.random.randn(len(t))
        
        return {
            'gaze_x': gaze_x,
            'gaze_y': gaze_y,
            'pupil_diameter': pupil_diameter
        }
    
    def _generate_synthetic_fnirs(self, stress_level: float) -> Dict:
        """Generate synthetic fNIRS data"""
        duration = 1.0
        fs = 10  # Lower sampling rate for fNIRS
        t = np.linspace(0, duration, int(duration * fs))
        
        # Activation in prefrontal cortex related to stress/attention
        activation_level = stress_level * 0.2
        
        fnirs_data = {
            'ch1': {
                'oxy_hb': activation_level + 0.05 * np.random.randn(len(t)),
                'deoxy_hb': -activation_level * 0.5 + 0.02 * np.random.randn(len(t))
            },
            'ch2': {
                'oxy_hb': activation_level * 0.8 + 0.04 * np.random.randn(len(t)),
                'deoxy_hb': -activation_level * 0.4 + 0.015 * np.random.randn(len(t))
            }
        }
        
        return fnirs_data
    
    def _process_accumulated_sensor_data(self):
        """Process accumulated sensor data"""
        # Collect data from the last processing interval
        sensor_data_list = []
        while not self.sensor_data_queue.empty():
            try:
                data = self.sensor_data_queue.get_nowait()
                sensor_data_list.append(data)
            except queue.Empty:
                break
        
        if not sensor_data_list:
            return
        
        # Combine data into processing window
        window_start = sensor_data_list[0]['timestamp']
        window_end = sensor_data_list[-1]['timestamp']
        
        # Aggregate sensor arrays
        window_data = {
            'start_time': window_start,
            'end_time': window_end,
            'ecg': np.concatenate([d['ecg'] for d in sensor_data_list]),
            'eda': np.concatenate([d['eda'] for d in sensor_data_list]),
            'eye_tracking': {
                'gaze_x': np.concatenate([d['eye_tracking']['gaze_x'] for d in sensor_data_list]),
                'gaze_y': np.concatenate([d['eye_tracking']['gaze_y'] for d in sensor_data_list]),
                'pupil_diameter': np.concatenate([d['eye_tracking']['pupil_diameter'] for d in sensor_data_list])
            },
            'fnirs': {
                'ch1': {
                    'oxy_hb': np.concatenate([d['fnirs']['ch1']['oxy_hb'] for d in sensor_data_list]),
                    'deoxy_hb': np.concatenate([d['fnirs']['ch1']['deoxy_hb'] for d in sensor_data_list])
                },
                'ch2': {
                    'oxy_hb': np.concatenate([d['fnirs']['ch2']['oxy_hb'] for d in sensor_data_list]),
                    'deoxy_hb': np.concatenate([d['fnirs']['ch2']['deoxy_hb'] for d in sensor_data_list])
                }
            }
        }
        
        # Get behavioral events in this window
        behavioral_events = self._get_behavioral_events_in_window(window_start, window_end)
        
        # Process with synchronized analyzer
        results = self.sensor_analyzer.analyze_time_window(window_data, behavioral_events)
        
        # Store results
        self.processed_results.append(results)
        
        # Log processing results
        self.data_collector.log_behavioral_event("sensor_processing_complete", 
                                                "processing", {
                                                    'window_duration': window_end - window_start,
                                                    'num_events': len(behavioral_events),
                                                    'stress_indicators': self._extract_stress_indicators(results)
                                                })
        
        print(f"Processed sensor window: {window_end - window_start:.1f}s, {len(behavioral_events)} events")
    
    def _get_behavioral_events_in_window(self, start_time: float, end_time: float) -> List[Dict]:
        """Get behavioral events that occurred in the specified time window"""
        # In a real system, this would query the behavior history
        # For simulation, create events based on current behavior
        events = []
        
        current_state = self.current_robot_state.get('behavioral_state', 'unknown')
        if current_state != 'idle':
            events.append({
                'timestamp': (start_time + end_time) / 2,
                'event_type': current_state,
                'robot_state': current_state
            })
        
        return events
    
    def _extract_stress_indicators(self, results: Dict) -> Dict:
        """Extract stress indicators from processing results"""
        stress_indicators = {}
        
        # HRV stress indicators
        if 'hrv' in results:
            hrv = results['hrv']
            stress_indicators['hrv_stress'] = (
                (hrv.get('lf_hf_ratio', 0) > 1.5) + 
                (hrv.get('rmssd', 100) < 20)
            )
        
        # EDA stress indicators
        if 'eda' in results:
            eda = results['eda']
            stress_indicators['eda_stress'] = (
                (eda.get('scr_frequency', 0) > 0.1) + 
                (eda.get('tonic_slope', 0) > 0)
            )
        
        # Eye tracking stress indicators
        if 'eye_tracking' in results:
            eye = results['eye_tracking']
            stress_indicators['eye_stress'] = (
                (eye.get('pupil_diameter_mean', 0) > 4.0) + 
                (eye.get('saccade_rate', 0) > 3.0)
            )
        
        return stress_indicators
    
    def _generate_study_report(self):
        """Generate final study report"""
        # Generate summary from all processed results
        summary = self.sensor_analyzer.generate_summary_report(self.processed_results)
        
        # Add study metadata
        study_report = {
            'session_info': {
                'session_id': self.config.session_id,
                'participant_id': self.config.participant_id,
                'condition': self.config.condition,
                'start_time': self.start_time,
                'duration': time.time() - self.start_time
            },
            'behavior_summary': self.behavior_controller.state_machine.behavior_history,
            'sensor_summary': summary,
            'configuration': {
                'behavior_params': self.behavior_controller.get_parameters().__dict__,
                'environment_size': self.config.environment_size,
                'data_collection_rate': self.config.data_collection_rate
            }
        }
        
        # Save report
        report_filename = f"study_reports/study_report_{self.config.session_id}.json"
        with open(report_filename, 'w') as f:
            json.dump(study_report, f, indent=2, default=str)
        
        print(f"Study report saved: {report_filename}")
    
    def adjust_behavior_parameters(self, **kwargs):
        """Allow real-time adjustment of behavior parameters"""
        if 'speed' in kwargs:
            self.parameter_interface.set_speed_parameters(**kwargs['speed'])
        
        if 'proximity' in kwargs:
            self.parameter_interface.set_proximity_thresholds(**kwargs['proximity'])
        
        if 'behavior' in kwargs:
            self.parameter_interface.set_behavioral_weights(**kwargs['behavior'])
        
        # Log parameter change
        self.data_collector.log_behavioral_event("parameter_adjustment", "adjusting", kwargs)

# Test and demonstration functions
def run_study_demo():
    """Run a demonstration of the complete study system"""
    print("=" * 60)
    print("Integrated Behavioral Study System Demonstration")
    print("=" * 60)
    
    # Configure study
    config = StudyConfiguration(
        session_id=f"demo_study_{int(time.time())}",
        participant_id="demo_participant",
        condition="baseline",
        study_duration=30.0,  # 30 seconds for demo
        data_collection_rate=10.0,  # Lower rate for demo
        sensor_processing_interval=5.0
    )
    
    # Initialize system
    study_system = IntegratedStudySystem(config)
    
    try:
        # Start study
        study_system.start_study()
        
        print("Study running... (press Ctrl+C to stop early)")
        
        # Simulate some parameter adjustments during the study
        time.sleep(10)
        print("Adjusting behavior parameters...")
        study_system.adjust_behavior_parameters(
            speed={'patrol': 0.5, 'approach': 0.3},
            proximity={'close': 1.5, 'very_close': 0.8}
        )
        
        # Wait for study completion
        while study_system.running:
            time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nStopping study early...")
        study_system.stop_study()
    
    print("\nDemo completed successfully!")

if __name__ == "__main__":
    run_study_demo() 