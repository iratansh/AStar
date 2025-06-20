#!/usr/bin/env python3
"""
Data Collection System for Unitree Go2 Behavioral Study
Synchronizes robot telemetry with EEG equipment and other physiological sensors
"""

import time
import json
import threading
import queue
import socket
import struct
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, asdict
import numpy as np
from datetime import datetime
import csv
import os

@dataclass
class RobotTelemetry:
    """Robot telemetry data structure"""
    timestamp: float
    position: tuple  # (x, y, z)
    orientation: tuple  # (roll, pitch, yaw)
    velocity: tuple  # (vx, vy, vz)
    behavioral_state: str
    target_worker_id: Optional[int]
    worker_distances: Dict[int, float]
    sensor_data: Dict[str, Any]
    movement_speed: float
    emergency_status: bool

@dataclass
class EventMarker:
    """Event marker for behavioral changes"""
    timestamp: float
    event_type: str
    event_data: Dict[str, Any]
    robot_state: str
    participant_id: Optional[str] = None

class TimestampSynchronizer:
    """Handles timestamp synchronization between different systems"""
    
    def __init__(self, reference_system: str = "robot"):
        self.reference_system = reference_system
        self.sync_offsets: Dict[str, float] = {}
        self.sync_history: List[Dict] = []
        self.last_sync_time = 0
        
    def add_sync_point(self, system_name: str, system_timestamp: float, 
                      reference_timestamp: float):
        """Add synchronization point between systems"""
        offset = reference_timestamp - system_timestamp
        self.sync_offsets[system_name] = offset
        
        sync_data = {
            'timestamp': time.time(),
            'system': system_name,
            'system_time': system_timestamp,
            'reference_time': reference_timestamp,
            'offset': offset
        }
        self.sync_history.append(sync_data)
        print(f"Sync point added for {system_name}: offset = {offset:.6f}s")
    
    def synchronize_timestamp(self, system_name: str, system_timestamp: float) -> float:
        """Convert system timestamp to synchronized timestamp"""
        if system_name in self.sync_offsets:
            return system_timestamp + self.sync_offsets[system_name]
        else:
            print(f"Warning: No sync offset for {system_name}")
            return system_timestamp
    
    def auto_sync_with_eeg(self, eeg_interface):
        """Automatic synchronization with EEG system"""
        # Send sync pulse to EEG
        robot_time = time.time()
        eeg_time = eeg_interface.send_sync_pulse()
        
        if eeg_time:
            self.add_sync_point("eeg", eeg_time, robot_time)
            return True
        return False

class DataLogger:
    """Main data logging system"""
    
    def __init__(self, session_id: str, participant_id: str = None):
        self.session_id = session_id
        self.participant_id = participant_id
        self.start_time = time.time()
        
        # Data queues for different streams
        self.robot_data_queue = queue.Queue()
        self.event_queue = queue.Queue()
        self.sensor_data_queue = queue.Queue()
        
        # File handles
        self.data_files = {}
        self.csv_writers = {}
        
        # Logging control
        self.logging_active = False
        self.logging_thread = None
        
        # Synchronization
        self.synchronizer = TimestampSynchronizer()
        
        # Create session directory
        self.session_dir = f"data_collection/{session_id}"
        os.makedirs(self.session_dir, exist_ok=True)
        
        print(f"Data logger initialized for session: {session_id}")
    
    def start_logging(self):
        """Start data logging process"""
        if self.logging_active:
            return
        
        self.logging_active = True
        self._initialize_files()
        
        # Start logging thread
        self.logging_thread = threading.Thread(target=self._logging_worker)
        self.logging_thread.daemon = True
        self.logging_thread.start()
        
        print("Data logging started")
    
    def stop_logging(self):
        """Stop data logging process"""
        if not self.logging_active:
            return
        
        self.logging_active = False
        
        if self.logging_thread:
            self.logging_thread.join(timeout=5.0)
        
        self._close_files()
        print("Data logging stopped")
    
    def _initialize_files(self):
        """Initialize CSV files for different data types"""
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Robot telemetry file
        robot_file = f"{self.session_dir}/robot_telemetry_{timestamp_str}.csv"
        self.data_files['robot'] = open(robot_file, 'w', newline='')
        robot_fields = ['timestamp', 'sync_timestamp', 'pos_x', 'pos_y', 'pos_z', 
                       'ori_roll', 'ori_pitch', 'ori_yaw', 'vel_x', 'vel_y', 'vel_z',
                       'behavioral_state', 'target_worker_id', 'movement_speed', 
                       'emergency_status', 'num_workers_detected']
        self.csv_writers['robot'] = csv.DictWriter(self.data_files['robot'], fieldnames=robot_fields)
        self.csv_writers['robot'].writeheader()
        
        # Event markers file
        event_file = f"{self.session_dir}/event_markers_{timestamp_str}.csv"
        self.data_files['events'] = open(event_file, 'w', newline='')
        event_fields = ['timestamp', 'sync_timestamp', 'event_type', 'robot_state', 
                       'participant_id', 'event_data']
        self.csv_writers['events'] = csv.DictWriter(self.data_files['events'], fieldnames=event_fields)
        self.csv_writers['events'].writeheader()
        
        # Worker interaction data
        interaction_file = f"{self.session_dir}/worker_interactions_{timestamp_str}.csv"
        self.data_files['interactions'] = open(interaction_file, 'w', newline='')
        interaction_fields = ['timestamp', 'sync_timestamp', 'worker_id', 'distance',
                            'proximity_level', 'robot_state', 'interaction_duration']
        self.csv_writers['interactions'] = csv.DictWriter(self.data_files['interactions'], 
                                                        fieldnames=interaction_fields)
        self.csv_writers['interactions'].writeheader()
    
    def _close_files(self):
        """Close all open files"""
        for file_handle in self.data_files.values():
            file_handle.close()
        self.data_files.clear()
        self.csv_writers.clear()
    
    def _logging_worker(self):
        """Main logging worker thread"""
        while self.logging_active:
            try:
                # Process robot telemetry
                while not self.robot_data_queue.empty():
                    telemetry = self.robot_data_queue.get_nowait()
                    self._log_robot_telemetry(telemetry)
                
                # Process events
                while not self.event_queue.empty():
                    event = self.event_queue.get_nowait()
                    self._log_event(event)
                
                time.sleep(0.01)  # 100Hz logging rate
                
            except Exception as e:
                print(f"Error in logging worker: {e}")
    
    def _log_robot_telemetry(self, telemetry: RobotTelemetry):
        """Log robot telemetry data"""
        sync_timestamp = self.synchronizer.synchronize_timestamp("robot", telemetry.timestamp)
        
        row_data = {
            'timestamp': telemetry.timestamp,
            'sync_timestamp': sync_timestamp,
            'pos_x': telemetry.position[0],
            'pos_y': telemetry.position[1],
            'pos_z': telemetry.position[2],
            'ori_roll': telemetry.orientation[0],
            'ori_pitch': telemetry.orientation[1],
            'ori_yaw': telemetry.orientation[2],
            'vel_x': telemetry.velocity[0],
            'vel_y': telemetry.velocity[1],
            'vel_z': telemetry.velocity[2],
            'behavioral_state': telemetry.behavioral_state,
            'target_worker_id': telemetry.target_worker_id,
            'movement_speed': telemetry.movement_speed,
            'emergency_status': telemetry.emergency_status,
            'num_workers_detected': len(telemetry.worker_distances)
        }
        
        self.csv_writers['robot'].writerow(row_data)
        self.data_files['robot'].flush()
    
    def _log_event(self, event: EventMarker):
        """Log event marker"""
        sync_timestamp = self.synchronizer.synchronize_timestamp("robot", event.timestamp)
        
        row_data = {
            'timestamp': event.timestamp,
            'sync_timestamp': sync_timestamp,
            'event_type': event.event_type,
            'robot_state': event.robot_state,
            'participant_id': event.participant_id or self.participant_id,
            'event_data': json.dumps(event.event_data)
        }
        
        self.csv_writers['events'].writerow(row_data)
        self.data_files['events'].flush()
    
    def log_robot_telemetry(self, telemetry: RobotTelemetry):
        """Add robot telemetry to logging queue"""
        if self.logging_active:
            self.robot_data_queue.put(telemetry)
    
    def log_event(self, event_type: str, event_data: Dict[str, Any], robot_state: str):
        """Add event marker to logging queue"""
        if self.logging_active:
            event = EventMarker(
                timestamp=time.time(),
                event_type=event_type,
                event_data=event_data,
                robot_state=robot_state,
                participant_id=self.participant_id
            )
            self.event_queue.put(event)
    
    def save_session_metadata(self, metadata: Dict[str, Any]):
        """Save session metadata"""
        metadata_file = f"{self.session_dir}/session_metadata.json"
        
        session_data = {
            'session_id': self.session_id,
            'participant_id': self.participant_id,
            'start_time': self.start_time,
            'end_time': time.time(),
            'duration': time.time() - self.start_time,
            'sync_history': self.synchronizer.sync_history,
            'metadata': metadata
        }
        
        with open(metadata_file, 'w') as f:
            json.dump(session_data, f, indent=2)

class RealTimeStreamer:
    """Real-time data streaming for live monitoring"""
    
    def __init__(self, port: int = 8888):
        self.port = port
        self.server_socket = None
        self.client_connections = []
        self.streaming_active = False
        self.stream_thread = None
        
        # Data buffers
        self.data_buffer = queue.Queue(maxsize=1000)
    
    def start_streaming(self):
        """Start real-time data streaming server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('localhost', self.port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)
            
            self.streaming_active = True
            self.stream_thread = threading.Thread(target=self._streaming_worker)
            self.stream_thread.daemon = True
            self.stream_thread.start()
            
            print(f"Real-time streaming started on port {self.port}")
            
        except Exception as e:
            print(f"Failed to start streaming: {e}")
    
    def stop_streaming(self):
        """Stop real-time data streaming"""
        self.streaming_active = False
        
        if self.stream_thread:
            self.stream_thread.join(timeout=5.0)
        
        if self.server_socket:
            self.server_socket.close()
        
        for client in self.client_connections:
            client.close()
        self.client_connections.clear()
        
        print("Real-time streaming stopped")
    
    def _streaming_worker(self):
        """Main streaming worker thread"""
        while self.streaming_active:
            try:
                # Accept new connections
                try:
                    client_socket, address = self.server_socket.accept()
                    self.client_connections.append(client_socket)
                    print(f"New client connected: {address}")
                except socket.timeout:
                    pass
                
                # Send data to clients
                while not self.data_buffer.empty():
                    data = self.data_buffer.get_nowait()
                    self._broadcast_data(data)
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.streaming_active:
                    print(f"Streaming error: {e}")
    
    def _broadcast_data(self, data: Dict):
        """Broadcast data to all connected clients"""
        message = json.dumps(data).encode('utf-8')
        message_length = struct.pack('!I', len(message))
        
        disconnected_clients = []
        
        for client in self.client_connections:
            try:
                client.send(message_length + message)
            except Exception:
                disconnected_clients.append(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.client_connections.remove(client)
            client.close()
    
    def stream_data(self, data: Dict):
        """Add data to streaming buffer"""
        if self.streaming_active and not self.data_buffer.full():
            self.data_buffer.put(data)

class EEGInterface:
    """Interface for EEG equipment synchronization"""
    
    def __init__(self, eeg_system_type: str = "generic"):
        self.eeg_system_type = eeg_system_type
        self.connected = False
        
    def connect(self) -> bool:
        """Connect to EEG system"""
        # Implementation would depend on specific EEG system
        # This is a placeholder
        print(f"Connecting to {self.eeg_system_type} EEG system...")
        self.connected = True
        return True
    
    def send_sync_pulse(self) -> Optional[float]:
        """Send synchronization pulse to EEG system"""
        if not self.connected:
            return None
        
        # Implementation would send actual sync pulse
        # Return EEG system timestamp
        return time.time()
    
    def send_event_marker(self, event_type: str, event_data: Dict = None):
        """Send event marker to EEG system"""
        if not self.connected:
            return
        
        # Implementation would send marker to EEG
        print(f"EEG Event Marker: {event_type}")

class DataCollectionSystem:
    """Main data collection system coordinator"""
    
    def __init__(self, session_id: str, participant_id: str = None):
        self.session_id = session_id
        self.participant_id = participant_id
        
        # Components
        self.logger = DataLogger(session_id, participant_id)
        self.streamer = RealTimeStreamer()
        self.eeg_interface = EEGInterface()
        
        # Interaction tracking
        self.current_interactions = {}
        self.interaction_start_times = {}
        
        print(f"Data collection system initialized for session: {session_id}")
    
    def start_collection(self):
        """Start complete data collection system"""
        print("Starting data collection system...")
        
        # Connect to EEG
        if self.eeg_interface.connect():
            # Perform synchronization
            self.logger.synchronizer.auto_sync_with_eeg(self.eeg_interface)
        
        # Start logging and streaming
        self.logger.start_logging()
        self.streamer.start_streaming()
        
        # Log session start event
        self.logger.log_event("session_start", {
            'session_id': self.session_id,
            'participant_id': self.participant_id
        }, "idle")
        
        print("Data collection system started")
    
    def stop_collection(self):
        """Stop complete data collection system"""
        print("Stopping data collection system...")
        
        # Log session end event
        self.logger.log_event("session_end", {
            'duration': time.time() - self.logger.start_time
        }, "idle")
        
        # Stop components
        self.logger.stop_logging()
        self.streamer.stop_streaming()
        
        # Save metadata
        metadata = {
            'total_interactions': len(self.current_interactions),
            'eeg_connected': self.eeg_interface.connected
        }
        self.logger.save_session_metadata(metadata)
        
        print("Data collection system stopped")
    
    def collect_robot_data(self, robot_state: Dict, sensor_data: Dict = None):
        """Collect robot telemetry data"""
        telemetry = RobotTelemetry(
            timestamp=time.time(),
            position=robot_state.get('position', (0, 0, 0)),
            orientation=robot_state.get('orientation', (0, 0, 0)),
            velocity=robot_state.get('velocity', (0, 0, 0)),
            behavioral_state=robot_state.get('behavioral_state', 'unknown'),
            target_worker_id=robot_state.get('target_worker_id'),
            worker_distances=robot_state.get('worker_distances', {}),
            sensor_data=sensor_data or {},
            movement_speed=robot_state.get('movement_speed', 0.0),
            emergency_status=robot_state.get('emergency_status', False)
        )
        
        # Log telemetry
        self.logger.log_robot_telemetry(telemetry)
        
        # Stream real-time data
        stream_data = {
            'type': 'robot_telemetry',
            'timestamp': telemetry.timestamp,
            'behavioral_state': telemetry.behavioral_state,
            'position': telemetry.position,
            'workers_detected': len(telemetry.worker_distances)
        }
        self.streamer.stream_data(stream_data)
    
    def log_behavioral_event(self, event_type: str, robot_state: str, event_data: Dict = None):
        """Log behavioral state change or event"""
        # Log to system
        self.logger.log_event(event_type, event_data or {}, robot_state)
        
        # Send to EEG
        self.eeg_interface.send_event_marker(event_type, event_data)
        
        # Stream event
        stream_data = {
            'type': 'behavioral_event',
            'timestamp': time.time(),
            'event_type': event_type,
            'robot_state': robot_state,
            'event_data': event_data
        }
        self.streamer.stream_data(stream_data)
        
        print(f"Behavioral event logged: {event_type} ({robot_state})")
    
    def track_worker_interaction(self, worker_id: int, distance: float, robot_state: str):
        """Track interactions with workers"""
        current_time = time.time()
        proximity_threshold = 2.0  # meters
        
        if distance <= proximity_threshold:
            if worker_id not in self.current_interactions:
                # Start new interaction
                self.current_interactions[worker_id] = current_time
                self.interaction_start_times[worker_id] = current_time
                
                self.log_behavioral_event("interaction_start", robot_state, {
                    'worker_id': worker_id,
                    'initial_distance': distance
                })
        else:
            if worker_id in self.current_interactions:
                # End interaction
                interaction_duration = current_time - self.current_interactions[worker_id]
                del self.current_interactions[worker_id]
                
                self.log_behavioral_event("interaction_end", robot_state, {
                    'worker_id': worker_id,
                    'duration': interaction_duration,
                    'final_distance': distance
                })

# Test function
def test_data_collection():
    """Test the data collection system"""
    print("Testing Data Collection System")
    print("=" * 50)
    
    # Initialize system
    session_id = f"test_session_{int(time.time())}"
    collector = DataCollectionSystem(session_id, "test_participant")
    
    # Start collection
    collector.start_collection()
    
    print("Simulating robot behavior and data collection...")
    
    # Simulate some robot behavior
    test_scenarios = [
        {"behavioral_state": "patrol", "position": (0, 0, 0.3), "movement_speed": 1.0},
        {"behavioral_state": "approach", "position": (1, 1, 0.3), "movement_speed": 0.5, "target_worker_id": 1},
        {"behavioral_state": "observe", "position": (2, 1.5, 0.3), "movement_speed": 0.1, "target_worker_id": 1},
        {"behavioral_state": "retreat", "position": (1, 1, 0.3), "movement_speed": 0.8},
    ]
    
    for i, scenario in enumerate(test_scenarios):
        print(f"Scenario {i+1}: {scenario['behavioral_state']}")
        
        # Add some worker distances
        scenario['worker_distances'] = {1: 2.0 - i * 0.3}  # Worker getting closer
        
        # Collect robot data
        collector.collect_robot_data(scenario)
        
        # Log behavioral event
        collector.log_behavioral_event(f"state_change_{scenario['behavioral_state']}", 
                                     scenario['behavioral_state'])
        
        # Track worker interaction
        if 'target_worker_id' in scenario:
            collector.track_worker_interaction(scenario['target_worker_id'], 
                                             scenario['worker_distances'][1], 
                                             scenario['behavioral_state'])
        
        time.sleep(0.5)  # Brief pause between scenarios
    
    print("Stopping data collection...")
    time.sleep(1)
    collector.stop_collection()
    
    print("\n" + "=" * 50)
    print("Data collection system test completed!")
    print(f"Data saved in: data_collection/{session_id}/")

if __name__ == "__main__":
    test_data_collection() 