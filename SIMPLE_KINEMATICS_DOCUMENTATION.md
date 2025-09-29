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
- **Link 2 Length (L2)**: 10.0 cm (elbow to end-effector)  
- **Maximum Reach**: 20.0 cm (L1 + L2)
- **Minimum Reach**: 0.0 cm (|L1 - L2|)
- **Coordinate System**: Origin at base, X-axis forward, Y-axis up

## Usage Instructions

### 1. Setup
1. Upload `main.ino` to your ESP32-S3
2. Open Serial Monitor at **115200 baud**
3. Wait for "Ready for commands!" message

### 2. Input Commands
The controller accepts **3 types of commands**:

#### A) Coordinate Movement (Inverse Kinematics)
```
12.5,8.3    → Move to X=12.5cm, Y=8.3cm
10,5        → Move to X=10cm, Y=5cm
15,0        → Move to X=15cm, Y=0cm
0,20        → Move to maximum Y reach
```

#### B) Individual Servo Control
```
S1,90       → Move Servo 1 (base/shoulder) to 90 degrees
S1,45       → Move Servo 1 to 45 degrees
S1,0        → Move Servo 1 to 0 degrees
```

```
S2,90       → Move Servo 2 (elbow) to 90 degrees  
S2,135      → Move Servo 2 to 135 degrees
S2,45       → Move Servo 2 to 45 degrees
```

### 3. System Response
#### For coordinate commands:
```
Moving to: (12.50, 8.30)
Joint angles: θ1=45.23°, θ2=67.89°
Servo commands: S1=112°, S2=158°
Movement complete!
```

#### For servo commands:
```
Servo 1 moved to: 90 degrees
```
or
```
Servo 2 moved to: 45 degrees
```

#### Mixed Usage Example:
```
S1,45       → Move base servo to 45°
S2,90       → Move elbow servo to 90° 
12,8        → Move to coordinates (12,8) using inverse kinematics
S1,0        → Return base servo to 0°
10,10       → Move to coordinates (10,10)
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
- Within maximum reach (≤ 20.0 cm)
- Above minimum reach (≥ 0.0 cm)  
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
15,0        → High X reach
0,15        → High Y reach
12,8        → Diagonal position
8,10        → Another diagonal
S1,90       → Move base to 90°
S2,45       → Move elbow to 45°
```

### Invalid Positions (Will be rejected)
```
25,0        → Beyond maximum reach (>20cm)
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
- **Coordinate Format**: `x,y` (comma-separated, no spaces)
- **Servo Format**: `S1,degrees` or `S2,degrees`
- **Terminator**: Newline character (\n)
- **Range**: Servo commands constrained to 0-180 degrees
- **Examples**: `12.5,8.3` or `S1,90` or `S2,45`

## Extensions
This basic controller can be extended with:
- Multiple elbow configurations (elbow-up/elbow-down)
- Path planning and smooth trajectories  
- GUI interface integration
- End-effector tools (gripper, pen, etc.)
- Real-time position feedback

## License
Open source - modify and use as needed for educational and research purposes.