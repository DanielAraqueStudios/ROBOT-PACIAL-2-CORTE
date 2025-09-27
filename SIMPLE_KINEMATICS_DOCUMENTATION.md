# ESP32-S3 2-DOF Robotic Arm Controller

## Overview
This project implements a 2-DOF (Degree of Freedom) robotic arm controller using an ESP32-S3 microcontroller with inverse kinematics calculations. The system allows you to input X,Y coordinates via UART and automatically calculates the required joint angles to position the end-effector.

## Hardware Requirements
- **ESP32-S3 Development Board**
- **2x Servo Motors** (SG90 or similar)
- **Power Supply** (5V for servos)
- **Jumper Wires**

## Pin Connections
```
ESP32-S3          Servo Motors
--------          ------------
Pin 9       →     Joint 1 (Base/Shoulder) Signal
Pin 10      →     Joint 2 (Elbow) Signal
5V          →     Servo VCC (both servos)
GND         →     Servo GND (both servos)
```

## Software Dependencies
- **ESP32Servo Library**: Install via Arduino IDE Library Manager
  - Go to: Tools → Manage Libraries → Search "ESP32Servo" → Install

## Robot Configuration
- **Link 1 Length (L1)**: 10.0 cm (shoulder to elbow)
- **Link 2 Length (L2)**: 8.0 cm (elbow to end-effector)  
- **Maximum Reach**: 18.0 cm (L1 + L2)
- **Minimum Reach**: 2.0 cm (|L1 - L2|)
- **Coordinate System**: Origin at base, X-axis forward, Y-axis up

## Usage Instructions

### 1. Setup
1. Upload `simple_kinematics.ino` to your ESP32-S3
2. Open Serial Monitor at **115200 baud**
3. Wait for "Ready for coordinates!" message

### 2. Input Format
Simply send X,Y coordinates separated by a comma:
```
12.5,8.3    → Move to X=12.5cm, Y=8.3cm
10,5        → Move to X=10cm, Y=5cm
15,0        → Move to X=15cm, Y=0cm
0,18        → Move to maximum Y reach
```

### 3. System Response
For each command, the system provides:
```
Moving to: (12.50, 8.30)
Joint angles: θ1=45.23°, θ2=67.89°
Servo commands: S1=112°, S2=158°
Movement complete!
```

## Mathematical Model

### Inverse Kinematics Equations
The system uses the following mathematical approach:

1. **Distance Calculation**:
   ```
   D = √(x² + y²)
   ```

2. **Joint 2 Angle (Elbow)**:
   ```
   cos(θ2) = (D² - L1² - L2²) / (2×L1×L2)
   θ2 = acos(cos(θ2))
   ```

3. **Joint 1 Angle (Shoulder)**:
   ```
   α = atan2(y, x)
   β = atan2(L2×sin(θ2), L1 + L2×cos(θ2))
   θ1 = α - β
   ```

### Workspace Validation
The system automatically validates that requested positions are:
- Within maximum reach (≤ 18.0 cm)
- Above minimum reach (≥ 2.0 cm)  
- Not at origin (singularity avoidance)

## Code Structure

### Main Functions
- `isPositionReachable()`: Validates if X,Y position is within workspace
- `calculateInverseKinematics()`: Computes joint angles for target position
- `kinematicToServoAngle()`: Converts kinematic angles to servo commands
- `moveToPosition()`: Complete movement sequence with feedback

### Key Variables
- `L1, L2`: Link lengths in centimeters
- `theta1, theta2`: Current joint angles in degrees
- `MAX_REACH, MIN_REACH`: Workspace boundaries

## Example Test Positions

### Safe Test Coordinates
```
10,5        → Mid-range position
15,0        → Maximum X reach
0,15        → Maximum Y reach
12,8        → Diagonal position
8,10        → Elbow-up configuration
```

### Invalid Positions (Will be rejected)
```
20,0        → Beyond maximum reach
1,1         → Below minimum reach  
0,0         → At origin (singularity)
```

## Troubleshooting

### Common Issues
1. **"Position not reachable!"**: Coordinate is outside workspace
2. **"No solution exists!"**: Mathematical calculation failed
3. **Servo not moving**: Check power supply and connections
4. **Incorrect movements**: Verify L1/L2 values match your robot

### Servo Calibration
If servos don't reach expected positions:
1. Adjust `kinematicToServoAngle()` scaling factors
2. Modify servo center positions in setup()
3. Check physical link lengths match L1/L2 values

## Communication Protocol
- **Baud Rate**: 115200
- **Format**: Plain text, comma-separated
- **Terminator**: Newline character (\n)
- **No command prefix required**: Just send coordinates directly

## Extensions
This basic controller can be extended with:
- Multiple elbow configurations (elbow-up/elbow-down)
- Path planning and smooth trajectories  
- GUI interface integration
- End-effector tools (gripper, pen, etc.)
- Real-time position feedback

## License
Open source - modify and use as needed for educational and research purposes.