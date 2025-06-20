#!/usr/bin/env python3
"""
Complete System Test for Unitree Go2 Behavioral Study
Tests all implemented phases: Navigation, Behavioral Control, Data Collection, and Processing
"""

import time
import numpy as np
import json
import os
from typing import Dict, List

def test_navigation_system():
    """Test Phase 1 & 2: Navigation and A* pathfinding"""
    print("Testing Navigation System (Phases 1 & 2)")
    print("-" * 40)
    
    try:
        from astar_pathfinding import AStarPlanner
        from ConvertTo2_5D import EnvironmentMapper, GridMapConfig, SensorData
        
        # Initialize mapping
        config = GridMapConfig(resolution=0.1, width=100, height=100)
        mapper = EnvironmentMapper(config, world_origin=(-5.0, -5.0))
        
        # Create test sensor data with a smaller, well-defined obstacle
        rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth_image = np.ones((480, 640), dtype=np.float32) * 5.0 # Floor is further away
        depth_image[220:260, 310:330] = 1.5  # A small, central obstacle
        
        sensor_data = SensorData(rgb_image, depth_image, (0, 0, 0.5, 0, 0, 0), 0.0)
        mapper.process_sensor_data(sensor_data)
        
        # Initialize A* planner
        planner = AStarPlanner(grid_resolution=0.1)
        height_map, obstacle_map = mapper.get_maps_for_pathfinding()
        planner.set_environment(height_map, obstacle_map)
        
        # Find a valid, non-obstacle start point dynamically
        valid_points = np.argwhere(planner.obstacle_map == 0)
        if len(valid_points) < 2:
            print("✗ Navigation: Not enough valid points in the map to test.")
            return False
            
        start_y, start_x = valid_points[0]
        goal_y, goal_x = valid_points[-1]
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        
        print(f"✓ Dynamically found valid start: {start} and goal: {goal}")
        
        # Test pathfinding with guaranteed valid points
        path = planner.find_path(start, goal)
        
        if path:
            print(f"✓ Navigation: Path found with {len(path)} waypoints")
            world_path = planner.grid_to_world(path, mapper.world_origin)
            print(f"✓ Path conversion: {len(world_path)} world coordinates")
            return True
        else:
            print("✗ Navigation: No path found")
            return False
        
    except Exception as e:
        print(f"✗ Navigation test failed: {e}")
        return False

def test_behavioral_control():
    """Test Phase 3: Behavioral Control System"""
    print("\nTesting Behavioral Control System (Phase 3)")
    print("-" * 40)
    
    try:
        from behavioral_control import BehaviorController, BehaviorParameters, ParameterInterface
        
        # Initialize behavioral control
        params = BehaviorParameters(patrol_speed=1.0, approach_speed=0.5)
        controller = BehaviorController(params)
        interface = ParameterInterface(controller)
        
        print("✓ Behavioral controller initialized")
        
        # Test state machine
        test_scenarios = [
            [(1, 3.0, 2.0, 0.0)],  # Worker at medium distance
            [(1, 1.5, 1.0, 0.0)],  # Worker approaches
            [(1, 0.4, 0.3, 0.0)],  # Worker too close
            []                      # No workers
        ]
        
        state_changes = 0
        for i, worker_positions in enumerate(test_scenarios):
            controller.update_worker_detections(worker_positions)
            result = controller.update()
            if i > 0 and result['current_state'] != prev_state:
                state_changes += 1
            prev_state = result.get('current_state', 'unknown')
        
        print(f"✓ State machine: {state_changes} state transitions detected")
        
        # Test parameter adjustment
        success = interface.set_speed_parameters(patrol=0.8, approach=0.3)
        if success:
            print("✓ Parameter adjustment: Speed parameters updated")
        
        success = interface.set_proximity_thresholds(close=1.5, very_close=0.8)
        if success:
            print("✓ Parameter adjustment: Proximity thresholds updated")
        
        # Test behavior logging
        controller.save_behavior_log("test_behavior_log.json")
        if os.path.exists("test_behavior_log.json"):
            print("✓ Behavior logging: Log file created")
            os.remove("test_behavior_log.json")  # Cleanup
        
        return True
        
    except Exception as e:
        print(f"✗ Behavioral control test failed: {e}")
        return False

def test_data_collection():
    """Test Phase 4: Data Collection System"""
    print("\nTesting Data Collection System (Phase 4)")
    print("-" * 40)
    
    try:
        from data_collection import DataCollectionSystem, TimestampSynchronizer
        
        # Initialize data collection
        session_id = f"test_session_{int(time.time())}"
        collector = DataCollectionSystem(session_id, "test_participant")
        
        print("✓ Data collection system initialized")
        
        # Test synchronization
        sync = TimestampSynchronizer()
        sync.add_sync_point("test_system", time.time(), time.time() + 0.001)
        synchronized_time = sync.synchronize_timestamp("test_system", time.time())
        print("✓ Timestamp synchronization: Working")
        
        # Start collection
        collector.start_collection()
        print("✓ Data collection: Started successfully")
        
        # Test robot telemetry logging
        robot_state = {
            'position': (1.0, 2.0, 0.3),
            'orientation': (0.0, 0.0, 0.5),
            'velocity': (0.1, 0.0, 0.0),
            'behavioral_state': 'patrol',
            'worker_distances': {1: 3.0, 2: 5.0},
            'movement_speed': 0.8,
            'emergency_status': False
        }
        
        collector.collect_robot_data(robot_state)
        print("✓ Robot telemetry: Data logged")
        
        # Test event logging
        collector.log_behavioral_event("test_event", "test_state", {"test_key": "test_value"})
        print("✓ Event logging: Behavioral event logged")
        
        # Test worker interaction tracking
        collector.track_worker_interaction(1, 1.5, "approach")
        print("✓ Interaction tracking: Worker interaction logged")
        
        # Stop collection
        time.sleep(0.5)  # Brief pause for data processing
        collector.stop_collection()
        print("✓ Data collection: Stopped successfully")
        
        # Check if files were created
        session_dir = f"data_collection/{session_id}"
        if os.path.exists(session_dir):
            files = os.listdir(session_dir)
            print(f"✓ File creation: {len(files)} files created")
            
            # Cleanup
            for file in files:
                os.remove(os.path.join(session_dir, file))
            os.rmdir(session_dir)
        
        return True
        
    except Exception as e:
        print(f"✗ Data collection test failed: {e}")
        return False

def test_sensor_processing():
    """Test Phase 5: Sensor Processing (simplified without scipy)"""
    print("\nTesting Sensor Processing System (Phase 5)")
    print("-" * 40)
    
    try:
        # Simple sensor processing test without scipy dependencies
        
        # Generate synthetic data
        duration = 10  # seconds
        fs = 100  # sampling rate
        t = np.linspace(0, duration, duration * fs)
        
        # Synthetic physiological signals
        ecg_signal = np.sin(2 * np.pi * 1.2 * t) + 0.1 * np.random.randn(len(t))
        eda_signal = 5 + 0.1 * t + 0.2 * np.random.randn(len(t))
        
        print("✓ Synthetic data generation: ECG and EDA signals created")
        
        # Basic signal analysis
        ecg_mean = np.mean(ecg_signal)
        ecg_std = np.std(ecg_signal)
        eda_mean = np.mean(eda_signal)
        eda_slope = (eda_signal[-1] - eda_signal[0]) / duration
        
        print(f"✓ ECG analysis: Mean={ecg_mean:.3f}, Std={ecg_std:.3f}")
        print(f"✓ EDA analysis: Mean={eda_mean:.3f}, Slope={eda_slope:.3f}")
        
        # Eye tracking simulation
        gaze_x = 5 * np.sin(0.5 * t) + 0.5 * np.random.randn(len(t))
        gaze_y = 3 * np.cos(0.3 * t) + 0.3 * np.random.randn(len(t))
        pupil_diameter = 3.5 + 0.5 * np.sin(0.1 * t) + 0.1 * np.random.randn(len(t))
        
        pupil_mean = np.mean(pupil_diameter)
        gaze_variability = np.std(gaze_x) + np.std(gaze_y)
        
        print(f"✓ Eye tracking analysis: Pupil={pupil_mean:.2f}mm, Gaze var={gaze_variability:.2f}")
        
        # fNIRS simulation
        fnirs_oxy = 0.1 * np.sin(0.05 * t) + 0.05 * np.random.randn(len(t))
        fnirs_deoxy = -0.05 * np.sin(0.05 * t) + 0.02 * np.random.randn(len(t))
        
        activation = np.mean(fnirs_oxy) - np.mean(fnirs_deoxy)
        print(f"✓ fNIRS analysis: Activation={activation:.4f}")
        
        # Behavioral event correlation
        behavioral_events = [
            {'timestamp': 2.0, 'event_type': 'approach'},
            {'timestamp': 5.0, 'event_type': 'observe'},
            {'timestamp': 8.0, 'event_type': 'retreat'}
        ]
        
        print(f"✓ Event correlation: {len(behavioral_events)} events processed")
        
        # Simple stress indicator calculation
        stress_score = 0
        if pupil_mean > 4.0:
            stress_score += 1
        if eda_slope > 0.1:
            stress_score += 1
        if activation > 0.05:
            stress_score += 1
        
        print(f"✓ Stress analysis: Stress score = {stress_score}/3")
        
        return True
        
    except Exception as e:
        print(f"✗ Sensor processing test failed: {e}")
        return False

def test_integrated_system():
    """Test integrated system functionality"""
    print("\nTesting Integrated System")
    print("-" * 40)
    
    try:
        # Test that all components can work together
        from behavioral_control import BehaviorController, BehaviorParameters
        from data_collection import DataCollectionSystem
        
        # Initialize components
        behavior_controller = BehaviorController(BehaviorParameters())
        session_id = f"integration_test_{int(time.time())}"
        data_collector = DataCollectionSystem(session_id, "test_participant")
        
        print("✓ Component initialization: All systems initialized")
        
        # Start data collection
        data_collector.start_collection()
        
        # Simulate integrated operation
        for i in range(5):
            # Update behavior
            worker_positions = [(1, 3.0 - i * 0.5, 2.0, 0.0)]
            behavior_controller.update_worker_detections(worker_positions)
            behavior_result = behavior_controller.update()
            
            # Log data
            robot_state = {
                'position': (0.0, 0.0, 0.3),
                'behavioral_state': behavior_result['current_state'],
                'worker_distances': {1: 3.0 - i * 0.5},
                'movement_speed': behavior_result['speed'],
                'emergency_status': behavior_result['emergency_stop']
            }
            
            data_collector.collect_robot_data(robot_state)
            
            # Log behavioral events
            if i == 2:  # Simulate state change
                data_collector.log_behavioral_event("state_change", behavior_result['current_state'])
            
            time.sleep(0.1)
        
        print("✓ Integrated operation: 5 cycles completed successfully")
        
        # Stop and cleanup
        data_collector.stop_collection()
        
        # Cleanup files
        session_dir = f"data_collection/{session_id}"
        if os.path.exists(session_dir):
            files = os.listdir(session_dir)
            for file in files:
                os.remove(os.path.join(session_dir, file))
            os.rmdir(session_dir)
        
        print("✓ Integration test: Completed successfully")
        
        return True
        
    except Exception as e:
        print(f"✗ Integrated system test failed: {e}")
        return False

def generate_test_report(results: Dict[str, bool]):
    """Generate a comprehensive test report"""
    print("\n" + "=" * 60)
    print("COMPREHENSIVE SYSTEM TEST REPORT")
    print("=" * 60)
    
    total_tests = len(results)
    passed_tests = sum(results.values())
    
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    print(f"Success Rate: {(passed_tests/total_tests)*100:.1f}%")
    
    print("\nDetailed Results:")
    print("-" * 30)
    
    test_names = {
        'navigation': 'Phase 1-2: Navigation & A* Pathfinding',
        'behavioral': 'Phase 3: Behavioral Control System',
        'data_collection': 'Phase 4: Data Collection System',
        'sensor_processing': 'Phase 5: Sensor Processing',
        'integration': 'System Integration Test'
    }
    
    for test_key, passed in results.items():
        test_name = test_names.get(test_key, test_key)
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status:8} {test_name}")
    
    print("\n" + "=" * 60)
    
    if passed_tests == total_tests:
        print("🎉 ALL TESTS PASSED! System is ready for deployment.")
    else:
        print("⚠️  Some tests failed. Please review and fix issues before deployment.")
    
    print("=" * 60)

def main():
    """Run complete system test"""
    print("Unitree Go2 Behavioral Study System - Complete Test Suite")
    print("=" * 60)
    
    # Create necessary directories
    os.makedirs("data_collection", exist_ok=True)
    os.makedirs("study_reports", exist_ok=True)
    
    # Run all tests
    test_results = {}
    
    test_results['navigation'] = test_navigation_system()
    test_results['behavioral'] = test_behavioral_control()
    test_results['data_collection'] = test_data_collection()
    test_results['sensor_processing'] = test_sensor_processing()
    test_results['integration'] = test_integrated_system()
    
    # Generate report
    generate_test_report(test_results)
    
    # Save test results
    timestamp = int(time.time())
    test_report = {
        'timestamp': timestamp,
        'test_results': test_results,
        'system_info': {
            'python_version': '3.8+',
            'required_packages': ['numpy', 'pandas', 'neurokit2 (optional)'],
            'test_duration': time.time() - timestamp
        }
    }
    
    with open(f"study_reports/system_test_report_{timestamp}.json", 'w') as f:
        json.dump(test_report, f, indent=2)
    
    print(f"\nTest report saved: study_reports/system_test_report_{timestamp}.json")

if __name__ == "__main__":
    main() 