# Webots Construction Site Behavioral Study

## Setup Instructions

### 1. Install Webots
- Download and install Webots from: https://cyberbotics.com/
- Ensure Webots is installed in the default location (/usr/local/webots on Linux/Mac)

### 2. Environment Setup
- Run this setup script: `python3 setup_webots_environment.py`
- Install Python dependencies: `pip3 install -r webots_requirements.txt`

### 3. Configure Webots Python Path
Add Webots Python controller library to your Python path:

**Linux/Mac:**
```bash
export PYTHONPATH=$PYTHONPATH:/usr/local/webots/lib/controller/python
```

**Windows:**
```cmd
set PYTHONPATH=%PYTHONPATH%;C:\Program Files\Webots\lib\controller\python
```

### 4. Running the Simulation

1. Open Webots
2. Open the world file: `worlds/construction_site.wbt`
3. Click the play button to start the simulation

### 5. Features

#### Robot Capabilities:
- **Navigation**: 2.5D A* pathfinding with obstacle avoidance
- **Sensor Suite**: RGB-D camera, Lidar, GPS, IMU, Compass
- **Behavioral Control**: State machine with patrol, approach, observe, retreat states
- **Data Collection**: Real-time logging of robot telemetry and worker interactions

#### Human Workers:
- **Realistic Movement**: Work zones, breaks, natural walking patterns
- **Three Workers**: Each with different work areas and behaviors
- **Dynamic Behavior**: Random work durations and movement patterns

#### Environment:
- **Construction Site**: Scaffolding, equipment, materials, barriers
- **Realistic Lighting**: Multiple light sources for realistic conditions
- **Safety Elements**: Cones, barriers, proper construction layout

### 6. Data Output

The simulation generates:
- `data_collection/`: Robot telemetry and interaction logs
- `study_reports/`: Analysis reports and session summaries
- `behavior_logs/`: Detailed behavioral state histories

### 7. Customization

#### Behavioral Parameters:
Edit `behavioral_control.py` to modify:
- Movement speeds
- Proximity thresholds
- State transition criteria

#### Environment:
Edit `construction_site.wbt` to modify:
- Object positions
- Worker starting locations
- Environment layout

#### Worker Behavior:
Edit `human_controller.py` to modify:
- Work zones
- Movement patterns
- Break frequencies

### 8. Troubleshooting

**Common Issues:**

1. **Python Import Errors:**
   - Ensure PYTHONPATH includes Webots controller library
   - Verify all behavioral study modules are in controller directories

2. **Sensor Data Issues:**
   - Check that all sensors are properly enabled
   - Verify camera resolution settings match expectations

3. **Navigation Problems:**
   - Check that environment mapping is working
   - Verify A* planner has valid start/goal points

4. **Controller Errors:**
   - Check Webots console for detailed error messages
   - Ensure all required Python packages are installed

### 9. Research Usage

This simulation is designed for human-robot interaction research:

- **Behavioral Analysis**: Study robot behavioral patterns and decision making
- **Physiological Correlation**: Sync with EEG/EMG equipment for human response analysis
- **Parameter Studies**: Test different behavioral parameters and their effects
- **Safety Research**: Analyze safe interaction distances and protocols

For more detailed information, see the main project documentation.
