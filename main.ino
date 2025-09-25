/*
 * ESP32-S3 2-DOF Robotic Arm Controller with Inverse Kinematics
 * Hardware: ESP32-S3 Development Board
 * Components: 2x Servomotors (SG90 or similar) for 2-DOF planar manipulator
 * Pin Connections:
 * - Joint 1 (Base): Pin 9    - Controls shoulder rotation
 * - Joint 2 (Elbow): Pin 10  - Controls elbow rotation
 * - Power: 5V and GND for servos
 * 
 * Functionality: 2    else if (command == "STATUS") {
      Serial.println("📡 Received status request");
      displayRobotStatus();
    }
    else if (command == "STOP") {
      Serial.println("📡 Received emergency stop command");
      stopAllServos();
    }
    else if (command.startsWith("CONFIG ")) {robotic arm with inverse kinematics calculations
 * Features: Workspace validation, singularity avoidance, multiple solution handling
 * 
 * REQUIRED LIBRARY: ESP32Servo
 * Install via: Arduino IDE -> Tools -> Manage Libraries -> Search "ESP32Servo" -> Install
 * 
 * KINEMATIC MODEL:
 * - Link 1 Length (L1): Shoulder to elbow length
 * - Link 2 Length (L2): Elbow to end-effector length
 * - Joint 1 (θ1): Base rotation angle (shoulder)
 * - Joint 2 (θ2): Elbow rotation angle
 * - Coordinate System: Origin at base, X-axis forward, Y-axis up
 */

#include <ESP32Servo.h>  // ESP32-compatible servo library
#include <math.h>        // Mathematical functions for kinematics

// ========================= ROBOT CONFIGURATION =========================
// Robotic Arm Physical Parameters (in centimeters)
const float L1 = 10.0;  // Link 1 length (shoulder to elbow) - ADJUST FOR YOUR ROBOT
const float L2 = 8.0;   // Link 2 length (elbow to end-effector) - ADJUST FOR YOUR ROBOT

// Servo Motor Objects
Servo joint1;  // Base/Shoulder joint servo
Servo joint2;  // Elbow joint servo

// Pin definitions for 2-DOF robotic arm
const int JOINT1_PIN = 9;   // Base joint (shoulder) connected to pin 9
const int JOINT2_PIN = 10;  // Elbow joint connected to pin 10

// Servo angle constraints (in degrees)
const int JOINT1_MIN = 0;    // Minimum angle for joint 1
const int JOINT1_MAX = 180;  // Maximum angle for joint 1
const int JOINT2_MIN = 0;    // Minimum angle for joint 2
const int JOINT2_MAX = 180;  // Maximum angle for joint 2

// Servo offset calibration (adjust for your specific servos)
const int JOINT1_OFFSET = 0;   // Offset to center joint 1 at 0 degrees (changed from 90)
const int JOINT2_OFFSET = 0;   // Offset to center joint 2 at 0 degrees (changed from 90)

// Servo control parameters
const int SERVO1_CENTER = 90;  // Center position for servo 1 (stops continuous rotation)
const int SERVO2_CENTER = 90;  // Center position for servo 2

// ========================= KINEMATIC VARIABLES =========================
// Current joint angles (in degrees)
float theta1 = 0.0;  // Joint 1 angle (base/shoulder)
float theta2 = 0.0;  // Joint 2 angle (elbow)

// Target position variables
float target_x = 0.0;  // Target X coordinate
float target_y = 0.0;  // Target Y coordinate

// Workspace limits
const float MAX_REACH = L1 + L2;  // Maximum reachable distance
const float MIN_REACH = abs(L1 - L2);  // Minimum reachable distance

// Solution selection preference
enum SolutionType {
  ELBOW_UP,    // Prefer elbow-up configuration
  ELBOW_DOWN,  // Prefer elbow-down configuration
  CLOSEST      // Choose solution closest to current position
};
SolutionType preferred_solution = ELBOW_UP;

// ========================= MATHEMATICAL CONSTANTS =========================
// Using ESP32 built-in mathematical constants
// PI, DEG_TO_RAD, and RAD_TO_DEG are already defined in Arduino.h
const float EPSILON = 0.001;  // Small value for numerical comparisons

// ========================= INVERSE KINEMATICS FUNCTIONS =========================

/*
 * INVERSE KINEMATICS MATHEMATICAL MODEL:
 * 
 * Given target position (x, y), calculate joint angles (θ1, θ2)
 * 
 * Equations:
 * x = L1*cos(θ1) + L2*cos(θ1 + θ2)
 * y = L1*sin(θ1) + L2*sin(θ1 + θ2)
 * 
 * Solution using Law of Cosines:
 * D = sqrt(x² + y²)  [Distance from origin to target]
 * 
 * cos(θ2) = (D² - L1² - L2²) / (2*L1*L2)
 * θ2 = ±acos(cos(θ2))  [Two solutions: elbow up/down]
 * 
 * θ1 = atan2(y, x) - atan2(L2*sin(θ2), L1 + L2*cos(θ2))
 */

struct InverseKinematicsSolution {
  bool valid;           // Solution validity flag
  float theta1_deg;     // Joint 1 angle in degrees
  float theta2_deg;     // Joint 2 angle in degrees
  float error;          // Solution error metric
  String config_type;   // "ELBOW_UP" or "ELBOW_DOWN"
};

// Workspace validation function
bool isPositionReachable(float x, float y) {
  float distance = sqrt(x*x + y*y);
  
  // Check if position is within workspace boundaries
  if (distance > MAX_REACH + EPSILON) {
    Serial.println("❌ Position beyond maximum reach");
    return false;
  }
  
  if (distance < MIN_REACH - EPSILON) {
    Serial.println("❌ Position too close (below minimum reach)");
    return false;
  }
  
  // Check for singularities (near-zero configurations)
  if (distance < EPSILON) {
    Serial.println("❌ Position at origin (singularity)");
    return false;
  }
  
  return true;
}

// Calculate inverse kinematics with multiple solutions
InverseKinematicsSolution calculateInverseKinematics(float target_x, float target_y, bool elbow_up) {
  InverseKinematicsSolution solution;
  solution.valid = false;
  solution.error = 1000.0;  // Initialize with high error
  
  // Step 1: Validate target position
  if (!isPositionReachable(target_x, target_y)) {
    Serial.printf("⚠️  Target position (%.2f, %.2f) is not reachable\n", target_x, target_y);
    return solution;
  }
  
  // Step 2: Calculate distance from origin to target
  float D = sqrt(target_x*target_x + target_y*target_y);
  
  // Step 3: Calculate θ2 using Law of Cosines
  float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  
  // Step 4: Check if solution exists (avoid domain errors in acos)
  if (cos_theta2 < -1.0 || cos_theta2 > 1.0) {
    Serial.printf("⚠️  No solution exists for position (%.2f, %.2f)\n", target_x, target_y);
    return solution;
  }
  
  // Step 5: Calculate θ2 (two possible solutions)
  float theta2_rad;
  if (elbow_up) {
    theta2_rad = acos(cos_theta2);  // Positive solution (elbow up)
    solution.config_type = "ELBOW_UP";
  } else {
    theta2_rad = -acos(cos_theta2); // Negative solution (elbow down)
    solution.config_type = "ELBOW_DOWN";
  }
  
  // Step 6: Calculate θ1 using atan2 for proper quadrant
  float alpha = atan2(target_y, target_x);
  float beta = atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));
  float theta1_rad = alpha - beta;
  
  // Step 7: Convert to degrees
  solution.theta1_deg = theta1_rad * RAD_TO_DEG;
  solution.theta2_deg = theta2_rad * RAD_TO_DEG;
  
  // Step 8: Normalize angles to [0, 360) range
  while (solution.theta1_deg < 0) solution.theta1_deg += 360;
  while (solution.theta1_deg >= 360) solution.theta1_deg -= 360;
  while (solution.theta2_deg < -180) solution.theta2_deg += 360;
  while (solution.theta2_deg > 180) solution.theta2_deg -= 360;
  
  // Step 9: Verify solution by forward kinematics
  float verify_x = L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad);
  float verify_y = L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad);
  solution.error = sqrt((verify_x - target_x)*(verify_x - target_x) + 
                       (verify_y - target_y)*(verify_y - target_y));
  
  // Step 10: Mark solution as valid if error is acceptable
  if (solution.error < 0.1) {  // 0.1 cm tolerance
    solution.valid = true;
  }
  
  return solution;
}

// Find best solution considering joint limits and preferences
InverseKinematicsSolution findBestSolution(float target_x, float target_y) {
  // Calculate both possible solutions
  InverseKinematicsSolution elbow_up = calculateInverseKinematics(target_x, target_y, true);
  InverseKinematicsSolution elbow_down = calculateInverseKinematics(target_x, target_y, false);
  
  // Check joint limits for both solutions
  bool elbow_up_valid = elbow_up.valid && 
    (elbow_up.theta1_deg >= JOINT1_MIN && elbow_up.theta1_deg <= JOINT1_MAX) &&
    (elbow_up.theta2_deg >= -90 && elbow_up.theta2_deg <= 90);
    
  bool elbow_down_valid = elbow_down.valid && 
    (elbow_down.theta1_deg >= JOINT1_MIN && elbow_down.theta1_deg <= JOINT1_MAX) &&
    (elbow_down.theta2_deg >= -90 && elbow_down.theta2_deg <= 90);
  
  // Select solution based on preferences and validity
  if (preferred_solution == ELBOW_UP && elbow_up_valid) {
    return elbow_up;
  } else if (preferred_solution == ELBOW_DOWN && elbow_down_valid) {
    return elbow_down;
  } else if (elbow_up_valid && !elbow_down_valid) {
    return elbow_up;
  } else if (!elbow_up_valid && elbow_down_valid) {
    return elbow_down;
  } else if (elbow_up_valid && elbow_down_valid) {
    // Both valid, choose based on smallest error or current position proximity
    if (elbow_up.error < elbow_down.error) {
      return elbow_up;
    } else {
      return elbow_down;
    }
  }
  
  // No valid solution found
  InverseKinematicsSolution invalid_solution;
  invalid_solution.valid = false;
  return invalid_solution;
}

// ========================= SERVO CONTROL FUNCTIONS =========================

// Emergency stop function for continuous rotation servos
void stopAllServos() {
  joint1.write(SERVO1_CENTER);  // Stop joint 1 (90 degrees stops continuous rotation)
  joint2.write(SERVO2_CENTER);  // Stop joint 2
  Serial.println("🛑 Emergency stop - All servos stopped");
}

// Convert kinematic angle to servo position
int kinematicToServoAngle(float kinematic_angle, int joint_number) {
  int servo_angle;
  
  if (joint_number == 1) {
    // Joint 1: Convert kinematic angle to servo angle
    // For continuous rotation servos, use 90 degrees as stop position
    servo_angle = (int)(kinematic_angle * 0.5) + SERVO1_CENTER;  // Scale down movement
  } else {
    // Joint 2: Convert relative elbow angle to servo angle
    servo_angle = (int)(kinematic_angle) + SERVO2_CENTER;
  }
  
  // Constrain to servo limits
  servo_angle = constrain(servo_angle, 0, 180);
  
  return servo_angle;
}

// Move robot to target position using inverse kinematics
bool moveToPosition(float x, float y) {
  Serial.printf("\n🎯 Moving to target position: (%.2f, %.2f) cm\n", x, y);
  
  // Calculate inverse kinematics
  InverseKinematicsSolution solution = findBestSolution(x, y);
  
  if (!solution.valid) {
    Serial.println("❌ No valid solution found for target position");
    return false;
  }
  
  // Display solution information
  Serial.printf("✅ Solution found (%s):\n", solution.config_type.c_str());
  Serial.printf("   Joint 1 (θ1): %.2f degrees\n", solution.theta1_deg);
  Serial.printf("   Joint 2 (θ2): %.2f degrees\n", solution.theta2_deg);
  Serial.printf("   Position error: %.4f cm\n", solution.error);
  
  // Convert to servo angles
  int servo1_angle = kinematicToServoAngle(solution.theta1_deg, 1);
  int servo2_angle = kinematicToServoAngle(solution.theta2_deg, 2);
  
  Serial.printf("   Servo 1 command: %d degrees\n", servo1_angle);
  Serial.printf("   Servo 2 command: %d degrees\n", servo2_angle);
  
  // Move servos to calculated positions
  joint1.write(servo1_angle);
  joint2.write(servo2_angle);
  
  // Update current joint angles
  theta1 = solution.theta1_deg;
  theta2 = solution.theta2_deg;
  
  Serial.println("🔄 Robot movement complete");
  return true;
}

// Display current robot status
void displayRobotStatus() {
  Serial.println("\n📊 ROBOT STATUS:");
  Serial.printf("   Current Joint 1 angle: %.2f degrees\n", theta1);
  Serial.printf("   Current Joint 2 angle: %.2f degrees\n", theta2);
  
  // Calculate current end-effector position using forward kinematics
  float current_x = L1 * cos(theta1 * DEG_TO_RAD) + L2 * cos((theta1 + theta2) * DEG_TO_RAD);
  float current_y = L1 * sin(theta1 * DEG_TO_RAD) + L2 * sin((theta1 + theta2) * DEG_TO_RAD);
  
  Serial.printf("   Current end-effector position: (%.2f, %.2f) cm\n", current_x, current_y);
  Serial.printf("   Workspace limits: Min=%.2f cm, Max=%.2f cm\n", MIN_REACH, MAX_REACH);
}

// ========================= SETUP FUNCTION =========================
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Allow serial monitor to connect
  
  Serial.println("🤖 ESP32-S3 2-DOF Robotic Arm Controller Starting...");
  Serial.println("====================================================");
  
  // Display robot configuration
  Serial.printf("🔧 Robot Configuration:\n");
  Serial.printf("   Link 1 Length (L1): %.2f cm\n", L1);
  Serial.printf("   Link 2 Length (L2): %.2f cm\n", L2);
  Serial.printf("   Maximum Reach: %.2f cm\n", MAX_REACH);
  Serial.printf("   Minimum Reach: %.2f cm\n", MIN_REACH);
  Serial.printf("   Workspace Area: %.2f cm²\n", PI * MAX_REACH * MAX_REACH);
  
  // Attach servos to pins
  joint1.attach(JOINT1_PIN);
  joint2.attach(JOINT2_PIN);
  Serial.printf("✅ Servos attached - Joint 1: Pin %d, Joint 2: Pin %d\n", JOINT1_PIN, JOINT2_PIN);
  
  // IMMEDIATE STOP - Critical for continuous rotation servos
  stopAllServos();
  delay(2000);  // Wait for servos to completely stop
  
  // Initialize robot to home position (straight up)
  Serial.println("\n🏠 Moving to HOME position...");
  theta1 = 0.0;   // Start at 0 degrees
  theta2 = 0.0;   // Straight configuration
  
  joint1.write(kinematicToServoAngle(theta1, 1));
  joint2.write(kinematicToServoAngle(theta2, 2));
  
  delay(2000);  // Allow time for servos to reach position
  
  displayRobotStatus();
  Serial.println("\n🚀 Robot initialized and ready for commands.");
  Serial.println("📡 Waiting for serial commands from GUI...");
  Serial.println("====================================================");
}

// ========================= MAIN LOOP FUNCTION =========================
void loop() {
  // Handle serial communication from GUI or other controllers
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Parse commands from GUI (format: "MOVE x,y" or "HOME")
    if (command.startsWith("MOVE ")) {
      String coords = command.substring(5);
      int comma = coords.indexOf(',');
      if (comma > 0) {
        float x = coords.substring(0, comma).toFloat();
        float y = coords.substring(comma + 1).toFloat();
        Serial.printf("📡 Received move command: (%.2f, %.2f)\n", x, y);
        moveToPosition(x, y);
      }
    }
    else if (command == "HOME") {
      Serial.println("📡 Received home command");
      // Stop any continuous rotation first
      joint1.write(SERVO1_CENTER);
      joint2.write(SERVO2_CENTER);
      delay(500);
      
      theta1 = 0.0;   // Reset to 0 degrees
      theta2 = 0.0;
      joint1.write(kinematicToServoAngle(theta1, 1));
      joint2.write(kinematicToServoAngle(theta2, 2));
      Serial.println("🏠 Returned to HOME position");
      displayRobotStatus();
    }
    else if (command == "STATUS") {
      Serial.println("� Received status request");
      displayRobotStatus();
    }
    else if (command.startsWith("CONFIG ")) {
      // Handle elbow configuration changes (ELBOW_UP or ELBOW_DOWN)
      String config = command.substring(7);
      if (config == "ELBOW_UP") {
        preferred_solution = ELBOW_UP;
        Serial.println("🔧 Configuration changed to ELBOW_UP");
      } else if (config == "ELBOW_DOWN") {
        preferred_solution = ELBOW_DOWN;
        Serial.println("🔧 Configuration changed to ELBOW_DOWN");
      }
    }
  }
  
  // Small delay to prevent excessive CPU usage
  delay(50);
}

// ========================= ADDITIONAL UTILITY FUNCTIONS =========================

/*
 * CUSTOM TARGET POSITION FUNCTION
 * 
 * Call this function to move to a specific target position:
 * Example usage:
 * 
 * void customMovement() {
 *   moveToPosition(12.5, 8.3);  // Move to specific X,Y coordinates
 * }
 * 
 * Replace the demo sequence in loop() with your custom movements
 */

/*
 * WORKSPACE ANALYSIS FUNCTIONS
 * 
 * Uncomment and use these for workspace analysis:
 * 
 * void printWorkspacePoints() {
 *   Serial.println("Workspace boundary points:");
 *   for (int angle = 0; angle < 360; angle += 30) {
 *     float x = MAX_REACH * cos(angle * DEG_TO_RAD);
 *     float y = MAX_REACH * sin(angle * DEG_TO_RAD);
 *     Serial.printf("Angle %d°: (%.2f, %.2f)\n", angle, x, y);
 *   }
 * }
 * 
 * void testSingularities() {
 *   Serial.println("Testing near-singularity positions:");
 *   moveToPosition(0.1, 0.1);    // Near origin
 *   delay(2000);
 *   moveToPosition(MAX_REACH-0.1, 0);  // Near max reach
 *   delay(2000);
 *   moveToPosition(-MAX_REACH+0.1, 0); // Near min reach
 * }
 */