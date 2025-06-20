# Realistic Construction Site Environment for Webots

This project provides a comprehensive, realistic construction site simulation environment for Webots featuring the official Unitree Go2 robot model, human workers using the Pedestrian proto, and advanced boundary management to prevent objects from running off the map.

## 🚀 **Key Improvements**

### **1. Official Robot Model**
- **Unitree Go2 Proto**: Uses the official Webots Go2 robot model instead of custom-built robots
- **Complete Sensor Suite**: RGB-D camera, Lidar, GPS, IMU, Compass
- **Realistic Physics**: Proper mass, inertia, and collision properties

### **2. Human Workers**
- **Pedestrian Proto**: Uses official Webots Pedestrian proto for realistic human models
- **Realistic Movement**: Natural walking patterns, work schedules, and break times
- **Safety Equipment**: Different colored helmets and work clothes

### **3. Realistic Construction Environment**
- **Professional Equipment**: Cranes, excavators, scaffolding, tool sheds
- **Building Structures**: Multi-story buildings under construction, warehouses
- **Safety Features**: Traffic cones, barriers, fencing around the perimeter
- **Material Storage**: Wooden boxes, pipes, tools, oil barrels

### **4. Boundary Management**
- **Boundary Supervisor**: Prevents all moving objects from leaving the construction site
- **Automatic Correction**: Objects that cross boundaries are automatically repositioned
- **Warning System**: Alerts when objects approach boundaries
- **Physics Reset**: Resets object velocities when corrections are applied

## 📋 **Prerequisites**

- **Webots R2023b or later**
- **Python 3.8+**
- **Required Python packages** (install via `pip install -r requirements.txt`):
  ```
  numpy
  opencv-python
  matplotlib
  scipy
  ```

## 🛠️ **Installation & Setup**

### **1. Webots Project Structure**
```
construction_site/
├── worlds/
│   └── construction_site.wbt
├── controllers/
│   ├── webots_robot_controller/
│   │   └── webots_robot_controller.py
│   ├── human_controller/
│   │   └── human_controller.py
│   └── boundary_supervisor/
│       └── boundary_supervisor.py
├── protos/
└── plugins/
```

### **2. Quick Setup**
1. **Clone/Download** the project files
2. **Run the setup script**:
   ```bash
   python setup_webots_environment.py
   ```
3. **Open Webots** and load `worlds/construction_site.wbt`
4. **Start the simulation** by clicking the play button

### **3. Manual Setup** (if setup script fails)
1. Create the directory structure above
2. Copy all controller files to their respective directories
3. Ensure all Python dependencies are installed
4. Open the world file in Webots

## 🎮 **Running the Simulation**

### **Starting the Simulation**
1. **Open Webots**
2. **Load the world file**: `File > Open World > construction_site.wbt`
3. **Click Play**: The simulation will start automatically
4. **Observe**: The Go2 robot will begin patrolling while humans work

### **What You'll See**
- **Go2 Robot**: Patrolling the construction site with realistic movement
- **Human Workers**: Three workers moving between work areas with realistic schedules
- **Construction Activity**: Realistic construction site with equipment and materials
- **Boundary Enforcement**: Objects stay within the defined construction area

### **Camera Controls**
- **Follow Robot**: The camera automatically follows the Go2 robot
- **Manual Control**: Use mouse to rotate view, scroll to zoom
- **Viewpoint Menu**: Select different preset viewpoints

## 🔧 **Configuration Options**

### **Robot Behavior**
Edit `webots_robot_controller.py` to modify:
- **Patrol patterns**: Change target generation logic
- **Sensor processing**: Adjust detection algorithms
- **Movement speed**: Modify locomotion parameters

### **Human Workers**
Edit `human_controller.py` to modify:
- **Work schedules**: Change task durations and types
- **Movement patterns**: Adjust walking speeds and paths
- **Work zones**: Modify assigned work areas

### **Construction Site**
Edit `construction_site.wbt` to modify:
- **Equipment placement**: Add/remove construction equipment
- **Site layout**: Change building positions and sizes
- **Lighting**: Adjust environmental lighting

### **Boundary Settings**
Edit `boundary_supervisor.py` to modify:
- **Boundary limits**: Change the construction site size
- **Warning distances**: Adjust when warnings are triggered
- **Correction forces**: Modify how objects are kept in bounds

## 📊 **Data Collection**

The system automatically collects:
- **Robot telemetry**: Position, orientation, sensor data
- **Human interactions**: Worker positions and activities
- **Behavioral events**: State changes and decisions
- **System performance**: Processing times and errors

Data is saved to the `data_collection/` directory in CSV format.

## 🐛 **Troubleshooting**

### **Common Issues**

**1. Objects Still Running Off Map**
- Check that `boundary_supervisor` controller is running
- Verify boundary limits in the supervisor code
- Ensure objects have proper physics properties

**2. Robot Not Moving**
- Verify the Go2 proto is properly loaded
- Check motor initialization in the robot controller
- Ensure GPS and compass sensors are working

**3. Humans Not Moving**
- Verify Pedestrian proto is loaded correctly
- Check human controller arguments in the world file
- Ensure work schedules are properly configured

**4. Performance Issues**
- Reduce Lidar resolution if simulation is slow
- Disable shadows in graphics settings
- Lower camera resolution if needed

### **Error Messages**

**"Warning: Motor [name] not found"**
- The Go2 proto may have different motor names
- Check the proto documentation for correct motor names

**"Could not import some modules"**
- Install missing Python dependencies
- Ensure all project files are in the correct directories

**"Boundary violation detected"**
- This is normal - the supervisor is working correctly
- Objects will be automatically repositioned

## 🔬 **Research Applications**

This environment is designed for:
- **Human-Robot Interaction Studies**: Observe how humans react to robot presence
- **Path Planning Research**: Test navigation algorithms in complex environments
- **Safety Analysis**: Study safety protocols in construction environments
- **Behavioral Studies**: Analyze robot behavior patterns
- **Sensor Fusion**: Test multi-modal sensor processing

## 📈 **Performance Metrics**

The system tracks:
- **Navigation efficiency**: Path planning success rates
- **Human detection accuracy**: Worker identification performance
- **Boundary compliance**: How well objects stay within bounds
- **System responsiveness**: Real-time performance metrics

## 🤝 **Contributing**

To contribute improvements:
1. Test your changes thoroughly
2. Ensure backward compatibility
3. Update documentation
4. Submit clear descriptions of changes

## 📄 **License**

This project builds upon the open-source Webots simulator and official robot protos. Please respect the licensing terms of all components used.

## 🆘 **Support**

For issues:
1. Check this README first
2. Review the troubleshooting section
3. Check Webots documentation for proto-specific issues
4. Ensure all dependencies are properly installed

## 🎯 **Next Steps**

Potential enhancements:
- **Advanced AI**: Implement machine learning for robot behavior
- **Weather Effects**: Add rain, wind, and lighting changes
- **More Equipment**: Additional construction vehicles and tools
- **Multi-Robot**: Support for multiple robots working together
- **VR Integration**: Virtual reality interface for immersive control

---

**Happy Simulating! 🤖🏗️** 