# ESP32-S3 Robot Control GUI Documentation

## 🎯 Overview

This professional PyQt6 application provides comprehensive control over your ESP32-S3 2-DOF robotic arm with advanced features including real-time visualization, inverse kinematics, and automated demo sequences.

## 🌟 Features

### **🎮 Manual Control**
- **Joint Control**: Direct angle control with sliders and spinboxes
- **Position Control**: X,Y coordinate input with workspace validation
- **Quick Positions**: Pre-defined positions (Home, Left, Right, Up, Down, Center)
- **Real-time Feedback**: Live visualization of robot configuration

### **🤖 Automatic Control** 
- **Demo Sequences**: Basic, Square, Circle, Wave, and Workspace demos
- **Trajectory Control**: Speed adjustment and smooth movement options
- **Sequence Editor**: Load, save, and play custom movement sequences
- **Path Planning**: Automated trajectory generation

### **📊 Advanced Visualization**
- **Workspace Display**: Interactive 2D workspace with boundary visualization
- **Robot Configuration**: Real-time forward kinematics display
- **Target Visualization**: Visual target positioning and path preview
- **Status Monitoring**: Live joint angles and end-effector position

### **🔌 Communication**
- **Serial Interface**: Robust ESP32 communication with auto-reconnection
- **Command Console**: Direct command sending with history
- **Data Parsing**: Automatic robot status extraction from serial data
- **Error Handling**: Comprehensive error detection and recovery

### **⚙️ Configuration**
- **Robot Parameters**: Adjustable link lengths and workspace limits
- **Servo Calibration**: Joint offset adjustment and calibration routines
- **Safety Settings**: Workspace validation and soft limit enforcement
- **Settings Persistence**: Automatic save/restore of user preferences

## 🚀 Installation

### **Method 1: Automatic Installation**
```bash
# Run the installer script
python install_requirements.py

# Or use the Windows batch file
run_gui.bat
```

### **Method 2: Manual Installation**
```bash
# Install required packages
pip install PyQt6>=6.4.0
pip install pyserial>=3.5
pip install matplotlib>=3.6.0
pip install numpy>=1.21.0

# Run the application
python robot_control_gui.py
```

### **System Requirements**
- **Python 3.8+** (recommended: Python 3.10+)
- **Windows 10/11** (tested), Linux, or macOS
- **4GB RAM** minimum, 8GB recommended
- **OpenGL support** for matplotlib rendering
- **Serial port access** for ESP32 communication

## 📱 User Interface Guide

### **🔌 Connection Tab**
![Connection Interface](docs/connection_tab.png)

**Serial Connection Group:**
- **Port Selection**: Auto-detects available COM ports
- **Refresh Button**: Updates port list
- **Baud Rate**: Configurable (default: 115200)
- **Connect/Disconnect**: Toggle connection status

**Serial Monitor Group:**
- **Output Display**: Real-time ESP32 communication log
- **Command Input**: Direct command sending to robot
- **Auto-scroll**: Automatic scrolling to latest messages

### **🎮 Manual Control Tab**
![Manual Control Interface](docs/manual_tab.png)

**Joint Control Group:**
- **Joint 1 (Base)**: 0-180° range with slider and spinbox
- **Joint 2 (Elbow)**: ±90° range with slider and spinbox
- **Real-time Update**: Live robot visualization updates

**Position Control Group:**
- **X Position**: -20.0 to 20.0 cm range
- **Y Position**: -5.0 to 20.0 cm range
- **Move to Position**: Execute inverse kinematics movement
- **Current Position**: Live end-effector coordinates

**Quick Positions Group:**
- **Pre-defined Targets**: Instant movement to common positions
- **One-click Operation**: Single button press movements

### **🤖 Auto Control Tab**
![Auto Control Interface](docs/auto_tab.png)

**Demo Sequences Group:**
- **Basic Demo**: Original ESP32 demo sequence
- **Geometric Paths**: Square, circle, and wave patterns
- **Workspace Test**: Boundary exploration demo
- **Stop Function**: Emergency demo termination

**Trajectory Control Group:**
- **Speed Control**: 100-2000ms interval adjustment
- **Smooth Movement**: Interpolated path planning
- **Real-time Adjustment**: Live speed modification

**Sequence Editor Group:**
- **JSON Format**: Standard movement sequence format
- **Load/Save**: File-based sequence management
- **Play Function**: Automated sequence execution

### **⚙️ Settings Tab**
![Settings Interface](docs/settings_tab.png)

**Robot Parameters Group:**
- **Link Lengths**: L1 and L2 physical dimensions
- **Live Update**: Real-time workspace recalculation
- **Precision Control**: 0.1cm adjustment resolution

**Servo Calibration Group:**
- **Joint Offsets**: Individual servo calibration values
- **Home Position**: Return to calibrated zero position
- **Calibration Wizard**: Guided setup process

**Safety Settings Group:**
- **Emergency Stop**: Immediate movement termination
- **Workspace Validation**: Boundary checking
- **Soft Limits**: Joint angle protection

### **📊 Visualization Panel**
![Visualization Interface](docs/visualization_panel.png)

**Workspace Canvas:**
- **Interactive Display**: Real-time robot configuration
- **Workspace Boundaries**: Min/max reach circles
- **Target Visualization**: Planned movement preview
- **Grid System**: Coordinate reference grid

**Robot Information:**
- **Live Status**: Current joint angles and position
- **Distance Display**: End-effector distance from origin
- **Workspace Validation**: Position validity indicator

## 🔧 Communication Protocol

### **Command Format**
The GUI communicates with the ESP32 using simple text commands:

```
JOINT1:<angle>        # Set joint 1 angle (0-180°)
JOINT2:<angle>        # Set joint 2 angle (-90-90°)
MOVE:<x>,<y>          # Move to X,Y position (cm)
HOME                  # Return to home position
DEMO:<type>           # Start demo sequence
STOP                  # Emergency stop
CALIBRATE:<o1>,<o2>   # Set joint offsets
```

### **Response Parsing**
The GUI automatically parses ESP32 responses for:
- Joint angle updates
- End-effector position data
- Error messages and warnings
- Demo sequence status

## 🎨 Dark Theme Customization

The application features a professional dark theme with:

### **Color Scheme**
- **Primary Background**: #2b2b2b (Dark gray)
- **Secondary Background**: #353535 (Medium gray)
- **Accent Color**: #4CAF50 (Material Green)
- **Text Color**: #ffffff (White)
- **Error Color**: #F44336 (Material Red)
- **Warning Color**: #FF9800 (Material Orange)

### **UI Components**
- **Modern Buttons**: Rounded corners with hover effects
- **Styled Sliders**: Custom handle design
- **Professional Inputs**: Focused border highlighting
- **Consistent Spacing**: Grid-based layout system

## 🔍 Advanced Features

### **Inverse Kinematics Integration**
```python
# The GUI implements the same mathematical model as the ESP32:
def calculate_inverse_kinematics(x, y):
    D = sqrt(x² + y²)
    cos_theta2 = (D² - L1² - L2²) / (2*L1*L2)
    theta2 = ±acos(cos_theta2)
    theta1 = atan2(y,x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
    return theta1, theta2
```

### **Workspace Validation**
```python
def is_position_reachable(x, y):
    distance = sqrt(x² + y²)
    return min_reach <= distance <= max_reach
```

### **Real-time Visualization**
- **Forward Kinematics**: Live robot pose calculation
- **Matplotlib Integration**: Professional plotting with Qt integration
- **Interactive Canvas**: Click-to-move functionality (planned feature)

## 🛠️ Troubleshooting

### **Connection Issues**
❌ **"Port is busy or doesn't exist"**
- Close all serial monitors and other applications
- Refresh port list
- Try different USB ports
- Check ESP32 power and connection

❌ **"Permission Error"**
- Run application as administrator (Windows)
- Check user permissions for serial port access
- Verify driver installation

### **GUI Issues**
❌ **"Import Error: No module named 'PyQt6'"**
```bash
pip install PyQt6
```

❌ **"Matplotlib backend error"**
```bash
pip install --upgrade matplotlib
```

❌ **"Serial communication timeout"**
- Check baud rate setting (115200)
- Verify ESP32 is running correct firmware
- Test with simple serial terminal first

### **Performance Issues**
❌ **Slow GUI response**
- Reduce update frequency in settings
- Close unnecessary background applications
- Check system resources

❌ **Visualization lag**
- Update graphics drivers
- Reduce plot complexity
- Disable smooth animations if needed

## 📈 Future Enhancements

### **Planned Features**
- **3D Visualization**: Full 3D robot rendering
- **Path Recording**: Record and replay manual movements
- **Network Control**: TCP/IP communication option
- **Multi-robot Support**: Control multiple robots simultaneously
- **AI Integration**: Machine learning-based path optimization
- **Voice Control**: Speech recognition for robot commands

### **Extensibility**
The modular design allows easy extension:
- **Custom Demo Sequences**: Add new automated patterns
- **Additional Visualizations**: Charts, graphs, and analysis tools
- **Plugin System**: Third-party feature integration
- **API Interface**: External application control

## 📚 Code Architecture

### **Main Components**
```
robot_control_gui.py
├── RobotControlGUI (Main Window)
├── SerialCommunication (Thread)
├── WorkspaceCanvas (Matplotlib Widget)
├── DarkTheme (Styling)
└── Utility Classes (Config, Position, etc.)
```

### **Design Patterns**
- **Model-View-Controller**: Clean separation of concerns
- **Observer Pattern**: Signal-slot communication
- **Thread Safety**: Proper concurrent data handling
- **Settings Persistence**: QSettings integration

## 🤝 Contributing

We welcome contributions! Areas for improvement:
- Additional demo sequences
- Enhanced visualizations
- Performance optimizations
- Cross-platform testing
- Documentation improvements

## 📄 License

This project is provided as educational material for robotics and embedded systems learning.

---

**Last Updated**: September 2025  
**Version**: 1.0  
**Compatibility**: ESP32-S3 2-DOF Robotic Arm