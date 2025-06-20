# Unitree Go2 Construction Site Navigation in Webots

A comprehensive robotics simulation system for navigating a Unitree Go2 quadruped robot through construction environments using 2.5D mapping and A* pathfinding in Webots.

## Features

- **2.5D Environment Mapping**: Converts RealSense D435i sensor data to navigation-ready grid maps
- **A* Pathfinding**: Advanced pathfinding with terrain traversability analysis and dynamic obstacle detection
- **Real-time Navigation**: Dynamic replanning when new obstacles are detected
- **Construction Site Simulation**: Realistic multi-level construction environment with scaffolding, equipment, and barriers
- **Robot Integration**: Full Unitree Go2 robot model with sensor integration

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Webots Robot  │───▶│  ConvertTo2.5D   │───▶│   A* Planner    │
│   (Go2 + RealSense) │    │  (Environment    │    │  (Pathfinding)  │
└─────────────────┘    │   Mapping)       │    └─────────────────┘
                       └──────────────────┘              │
                                ▲                        ▼
                                │                ┌─────────────────┐
                                │                │ Robot Controller│
                                └────────────────│ (Navigation)    │
                                                 └─────────────────┘
```

## Files Overview

### Phase 1-2: Navigation & Environment Mapping

- **`A*.py` / `astar_pathfinding.py`**: A* pathfinding algorithm with 2.5D terrain support
  - Handles slope constraints and step height limitations
  - Dynamic obstacle detection and avoidance
  - Path smoothing and optimization

- **`ConvertTo2_5D.py`**: Environment mapping from sensor data
  - RealSense D435i depth image processing
  - Point cloud to grid map conversion
  - Traversability analysis
  - Dynamic obstacle detection

- **`robot_controller.py`**: Main Webots robot controller
  - Integrates mapping and pathfinding systems
  - Robot movement control
  - Navigation state machine

### Phase 3: Behavioral Control System

- **`behavioral_control.py`**: Modular behavioral parameter control system
  - Robot movement speed control
  - Proximity thresholds to workers
  - Surveillance patterns and behaviors
  - State machine implementation (patrol, approach, observe, retreat)
  - Real-time parameter adjustment interface
  - Safety constraints and limits

### Phase 4: Data Collection System

- **`data_collection.py`**: Comprehensive data collection and synchronization
  - Robot telemetry logging
  - Timestamp synchronization with EEG equipment
  - Event markers for behavioral changes
  - Real-time data streaming
  - Worker interaction tracking

### Phase 5: Sensor Processing

- **`sensor_processing.py`**: Multi-modal sensor data processing
  - **EmotiBit (HRV/EDA)**: NeuroKit2 processing for RMSSD, pNN50, LF/HF ratio, SCR peaks
  - **Eye-tracking**: Pupil diameter, fixation patterns, saccade metrics
  - **fNIRS**: Neural activation analysis in prefrontal cortex
  - Time-window analysis synchronized with robot behavioral events

### Integration & Testing

- **`integrated_system.py`**: Complete integrated behavioral study system
- **`test_complete_system.py`**: Comprehensive test suite for all components
- **`demo.py`**: Standalone demonstration of core navigation functionality

## Installation

### Prerequisites

1. **Webots R2023b or later**
   - Download from [Cyberbotics website](https://cyberbotics.com/)
   - Ensure Python controller support is enabled

2. **Python 3.8 or later**

3. **Required Python packages**:
   ```bash
   pip install -r requirements.txt
   ```

### Setup

1. **Clone or download the project files**:
   ```
   A*/
   ├── A*.py
   ├── ConvertTo2.5D.py
   ├── robot_controller.py
   ├── requirements.txt
   └── README.md
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Webots World Setup

### Phase 1: Creating the Construction Site World

1. **Open Webots** and create a new world file

2. **Add Construction Environment Elements**:
   ```
   - Ground plane with varied terrain
   - Scaffolding structures (using Box and Cylinder nodes)
   - Construction equipment (cranes, excavators)
   - Material piles and barriers
   - Worker avatars with basic movement
   ```

3. **Configure Lighting**:
   - Add DirectionalLight for sun simulation
   - Add PointLight sources for work areas
   - Set realistic ambient lighting

### Phase 2: Unitree Go2 Robot Setup

1. **Import Go2 Robot**:
   - Add the Unitree Go2 PROTO to your world
   - Position at starting location (0, 0, 0.3)

2. **Add Sensors**:
   ```
   Camera {
     name "camera"
     width 640
     height 480
     fieldOfView 1.2
   }
   
   RangeFinder {
     name "range_finder"
     width 640
     height 480
     maxRange 10
     fieldOfView 1.2
   }
   
   GPS {
     name "gps"
   }
   
   Compass {
     name "compass"
   }
   ```

3. **Set Controller**:
   ```
   controller "robot_controller"
   ```

## Usage

### Basic Navigation

1. **Start Webots simulation**
2. **Load your construction site world**
3. **Run the robot controller**:
   ```python
   # The controller will automatically:
   # 1. Initialize sensors and navigation systems
   # 2. Start mapping the environment
   # 3. Navigate through predefined waypoints
   # 4. Avoid dynamic obstacles
   ```

### Custom Navigation Goals

Modify the goals in `robot_controller.py`:

```python
# Example navigation goals (you can modify these)
goals = [
    (10.0, 0.0),   # Move 10m forward
    (10.0, 10.0),  # Turn and move to corner
    (0.0, 10.0),   # Move to another corner
    (0.0, 0.0)     # Return to start
]
```

### Configuration Parameters

#### Grid Mapping Configuration
```python
config = GridMapConfig(
    resolution=0.1,              # 10cm per grid cell
    width=1000,                  # 100m x 100m area
    height=1000,
    max_height=5.0,              # Maximum height to consider
    min_height=-2.0,             # Minimum height (for holes)
    robot_height=0.3,            # Robot body height
    traversability_threshold=0.3  # Maximum slope (rise/run)
)
```

#### A* Pathfinding Parameters
```python
planner = AStarPlanner(
    grid_resolution=0.1,    # Match mapping resolution
    max_slope=0.3,          # Maximum traversable slope
    max_step_height=0.15,   # Maximum step height
    robot_radius=0.3        # Robot radius for collision
)
```

## Advanced Features

### Dynamic Obstacle Detection

The system automatically detects moving obstacles (workers, equipment) and replans paths:

```python
# Dynamic obstacles are detected by comparing occupancy maps
dynamic_obstacles = mapper.detect_dynamic_obstacles(sensor_data, previous_occupancy)

# Planner automatically updates and replans
if dynamic_obstacles:
    planner.clear_dynamic_obstacles()
    for obs_x, obs_y in dynamic_obstacles:
        planner.add_dynamic_obstacle(obs_x, obs_y, radius=3)
```

### Path Smoothing

Paths are automatically smoothed to reduce sharp turns:

```python
smoothed_path = planner.smooth_path(path, smoothing_factor=0.5)
```

### Map Persistence

Save and load environment maps:

```python
# Save current maps
mapper.save_maps("construction_site_map.json")

# Load previously saved maps
mapper.load_maps("construction_site_map.json")
```

## Testing

### Unit Tests

Test individual components:

```python
# Test A* pathfinding
python A*.py

# Test environment mapping
python ConvertTo2.5D.py
```

### Integration Testing

1. Create a simple test world with basic obstacles
2. Run the robot controller
3. Verify the robot successfully navigates around obstacles
4. Test dynamic obstacle detection by moving objects during simulation

## Performance Optimization

### Grid Resolution Trade-offs
- **Higher resolution (0.05m)**: More accurate but slower
- **Lower resolution (0.2m)**: Faster but less precise

### Update Frequencies
- **Mapping**: Every sensor frame (30-60 Hz)
- **Pathfinding**: On-demand when goal changes or obstacles detected
- **Dynamic obstacles**: Every 1-2 seconds

## Troubleshooting

### Common Issues

1. **"No path found" errors**:
   - Check start/goal positions are in traversable areas
   - Verify obstacle map isn't too restrictive
   - Increase grid resolution for better path finding

2. **Robot not moving**:
   - Verify motor names match your Go2 model
   - Check controller is properly loaded
   - Ensure gait generation is working

3. **Sensor data issues**:
   - Verify camera and range_finder devices are enabled
   - Check sensor mounting positions and orientations
   - Ensure proper coordinate transformations

### Debug Mode

Enable debug output:

```python
# Add debug prints in robot_controller.py
print(f"Current position: {self.current_position}")
print(f"Target waypoint: {target_x}, {target_y}")
print(f"Distance to target: {distance}")
```

## Future Enhancements

- **Multi-robot coordination**: Support for multiple robots
- **Machine learning integration**: Learn optimal paths from experience
- **Advanced gait control**: More sophisticated walking patterns
- **Semantic mapping**: Object recognition and semantic understanding
- **Human-robot interaction**: Safe navigation around workers

## License

This project is open source. Feel free to modify and distribute according to your needs.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Support

For questions or issues:
1. Check the troubleshooting section
2. Review Webots documentation
3. Open an issue with detailed error information 