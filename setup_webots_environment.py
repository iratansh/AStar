#!/usr/bin/env python3
"""
Setup script for Webots Construction Site Behavioral Study Environment
Helps configure and run the simulation
"""

import os
import shutil
import sys
from pathlib import Path

def create_webots_project_structure():
    """Create the necessary Webots project structure"""
    print("Setting up Webots project structure...")
    
    # Create directories
    directories = [
        "worlds",
        "controllers/robot_controller",
        "controllers/human_controller",
        "controllers/boundary_supervisor",
        "protos",
        "plugins"
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"✓ Created directory: {directory}")

def copy_controller_files():
    """Copy controller files to Webots controllers directory"""
    print("\nCopying controller files...")
    
    # Files to copy to robot_controller directory
    robot_controller_files = [
        "webots_robot_controller.py",
        "behavioral_control.py",
        "data_collection.py",
        "sensor_processing.py",
        "astar_pathfinding.py",
        "ConvertTo2_5D.py",
        "integrated_system.py"
    ]
    
    # Copy robot controller files
    for file in robot_controller_files:
        if os.path.exists(file):
            shutil.copy2(file, "controllers/robot_controller/")
            print(f"✓ Copied {file} to robot controller")
        else:
            print(f"⚠ Warning: {file} not found")
    
    # Rename the main controller file
    if os.path.exists("controllers/robot_controller/webots_robot_controller.py"):
        shutil.move(
            "controllers/robot_controller/webots_robot_controller.py",
            "controllers/robot_controller/robot_controller.py"
        )
        print("✓ Renamed main controller file")
    
    # Copy human controller
    if os.path.exists("human_controller.py"):
        shutil.copy2("human_controller.py", "controllers/human_controller/")
        print("✓ Copied human controller")
    
    # Copy boundary supervisor
    if os.path.exists("controllers/boundary_supervisor/boundary_supervisor.py"):
        print("✓ Boundary supervisor already in correct location")
    else:
        print("⚠ Warning: boundary_supervisor.py not found")

def copy_world_file():
    """Copy world file to worlds directory"""
    print("\nCopying world file...")
    
    if os.path.exists("construction_site.wbt"):
        shutil.copy2("construction_site.wbt", "worlds/")
        print("✓ Copied construction_site.wbt to worlds directory")
    else:
        print("⚠ Warning: construction_site.wbt not found")

def create_makefile():
    """Create Makefile for controllers"""
    print("\nCreating Makefiles...")
    
    # Robot controller Makefile
    robot_makefile_content = """# Webots Makefile for Robot Controller
null :

WEBOTS_HOME_PATH = /usr/local/webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
"""
    
    with open("controllers/robot_controller/Makefile", "w") as f:
        f.write(robot_makefile_content)
    print("✓ Created robot controller Makefile")
    
    # Human controller Makefile
    human_makefile_content = """# Webots Makefile for Human Controller
null :

WEBOTS_HOME_PATH = /usr/local/webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
"""
    
    with open("controllers/human_controller/Makefile", "w") as f:
        f.write(human_makefile_content)
    print("✓ Created human controller Makefile")
    
    # Boundary supervisor Makefile
    boundary_makefile_content = """# Webots Makefile for Boundary Supervisor
null :

WEBOTS_HOME_PATH = /usr/local/webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
"""
    
    with open("controllers/boundary_supervisor/Makefile", "w") as f:
        f.write(boundary_makefile_content)
    print("✓ Created boundary supervisor Makefile")

def create_runtime_ini():
    """Create runtime.ini file for Python controllers"""
    print("\nCreating runtime.ini files...")
    
    # Robot controller runtime.ini
    robot_runtime_content = """[python]
command = python3 robot_controller.py
"""
    
    with open("controllers/robot_controller/runtime.ini", "w") as f:
        f.write(robot_runtime_content)
    print("✓ Created robot controller runtime.ini")
    
    # Human controller runtime.ini
    human_runtime_content = """[python]
command = python3 human_controller.py
"""
    
    with open("controllers/human_controller/runtime.ini", "w") as f:
        f.write(human_runtime_content)
    print("✓ Created human controller runtime.ini")
    
    # Boundary supervisor runtime.ini
    boundary_runtime_content = """[python]
command = python3 boundary_supervisor.py
"""
    
    with open("controllers/boundary_supervisor/runtime.ini", "w") as f:
        f.write(boundary_runtime_content)
    print("✓ Created boundary supervisor runtime.ini")

def create_requirements():
    """Create requirements file for the Webots project"""
    print("\nCreating Webots requirements...")
    
    webots_requirements = """# Webots Construction Site Behavioral Study Requirements
numpy>=1.21.0
opencv-python>=4.5.0
pandas>=1.3.0
neurokit2>=0.2.0

# Note: Webots Python API is included with Webots installation
# Ensure PYTHONPATH includes Webots controller library:
# export PYTHONPATH=$PYTHONPATH:/usr/local/webots/lib/controller/python
"""
    
    with open("webots_requirements.txt", "w") as f:
        f.write(webots_requirements)
    print("✓ Created webots_requirements.txt")

def create_readme():
    """Create README for Webots setup"""
    print("\nCreating Webots README...")
    
    readme_content = """# Webots Construction Site Behavioral Study

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
set PYTHONPATH=%PYTHONPATH%;C:\\Program Files\\Webots\\lib\\controller\\python
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
"""
    
    with open("WEBOTS_README.md", "w") as f:
        f.write(readme_content)
    print("✓ Created WEBOTS_README.md")

def verify_setup():
    """Verify the setup is complete"""
    print("\n" + "="*50)
    print("VERIFYING SETUP")
    print("="*50)
    
    required_files = [
        "worlds/construction_site.wbt",
        "controllers/robot_controller/robot_controller.py",
        "controllers/human_controller/human_controller.py",
        "controllers/robot_controller/behavioral_control.py",
        "controllers/robot_controller/data_collection.py",
        "controllers/robot_controller/astar_pathfinding.py",
        "controllers/robot_controller/ConvertTo2_5D.py"
    ]
    
    missing_files = []
    for file in required_files:
        if os.path.exists(file):
            print(f"✓ {file}")
        else:
            print(f"✗ {file} - MISSING")
            missing_files.append(file)
    
    if missing_files:
        print(f"\n⚠ Warning: {len(missing_files)} files are missing")
        print("Please ensure all source files are present before running setup")
    else:
        print(f"\n🎉 Setup complete! All {len(required_files)} required files are present.")
        print("\nNext steps:")
        print("1. Install Webots from https://cyberbotics.com/")
        print("2. Set PYTHONPATH to include Webots controller library")
        print("3. Install Python dependencies: pip3 install -r webots_requirements.txt")
        print("4. Open Webots and load worlds/construction_site.wbt")
        print("5. See WEBOTS_README.md for detailed instructions")

def main():
    """Main setup function"""
    print("Webots Construction Site Behavioral Study Setup")
    print("="*50)
    
    # Check if we're in the right directory
    if not os.path.exists("behavioral_control.py"):
        print("Error: Please run this script from the main project directory")
        print("The directory should contain behavioral_control.py and other project files")
        sys.exit(1)
    
    try:
        create_webots_project_structure()
        copy_controller_files()
        copy_world_file()
        create_makefile()
        create_runtime_ini()
        create_requirements()
        create_readme()
        verify_setup()
        
    except Exception as e:
        print(f"Error during setup: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 