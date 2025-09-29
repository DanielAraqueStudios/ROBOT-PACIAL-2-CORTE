#include <ESP32Servo.h>/*

#include <math.h> * ESP32-S3 2-DOF Robotic Arm Controller with Inverse Kinematics

 * Hardware: ESP32-S3 Development Board

Servo joint1; * Components: 2x Servomotors (SG90 or similar) for 2-DOF planar manipulator

Servo joint2; * Pin Connections:

 * - Joint 1 (Base): Pin 9    - Controls shoulder rotation

const int JOINT1_PIN = 9; * - Joint 2 (Elbow): Pin 10  - Controls elbow rotation

const int JOINT2_PIN = 10; * - Power: 5V and GND for servos

 * 

const float L1 = 10.0; * Functionality: 2    else if (command == "STATUS") {

const float L2 = 10.0;      Serial.println("📡 Received status request");

const float MAX_REACH = L1 + L2;      displayRobotStatus();

const float MIN_REACH = abs(L1 - L2);    }

    else if (command == "STOP") {

float theta1 = 0.0;      Serial.println("📡 Received emergency stop command");

float theta2 = 0.0;      stopAllServos();

    }

bool isPositionReachable(float x, float y) {    else if (command.startsWith("CONFIG ")) {robotic arm with inverse kinematics calculations

  float distance = sqrt(x*x + y*y); * Features: Workspace validation, singularity avoidance, multiple solution handling

  return (distance <= MAX_REACH && distance >= MIN_REACH && distance > 0.001); * 

} * REQUIRED LIBRARY: ESP32Servo

 * Install via: Arduino IDE -> Tools -> Manage Libraries -> Search "ESP32Servo" -> Install

bool calculateInverseKinematics(float target_x, float target_y, float &theta1_deg, float &theta2_deg) { * 

  if (!isPositionReachable(target_x, target_y)) { * KINEMATIC MODEL:

    Serial.println("Position not reachable!"); * - Link 1 Length (L1): Shoulder to elbow length

    return false; * - Link 2 Length (L2): Elbow to end-effector length

  } * - Joint 1 (θ1): Base rotation angle (shoulder)

   * - Joint 2 (θ2): Elbow rotation angle

  float D = sqrt(target_x*target_x + target_y*target_y); * - Coordinate System: Origin at base, X-axis forward, Y-axis up

  float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0 * L1 * L2); */

  

  if (cos_theta2 < -1.0 || cos_theta2 > 1.0) {#include <ESP32Servo.h>  // ESP32-compatible servo library

    Serial.println("No solution exists!");#include <math.h>        // Mathematical functions for kinematics

    return false;

  }// ========================= ROBOT CONFIGURATION =========================

  // Robotic Arm Physical Parameters (in centimeters)

  float theta2_rad = acos(cos_theta2);const float L1 = 10.0;  // Link 1 length (shoulder to elbow) - ADJUST FOR YOUR ROBOT

  float alpha = atan2(target_y, target_x);const float L2 = 8.0;   // Link 2 length (elbow to end-effector) - ADJUST FOR YOUR ROBOT

  float beta = atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));

  float theta1_rad = alpha - beta;// Servo Motor Objects

  Servo joint1;  // Base/Shoulder joint servo

  theta1_deg = theta1_rad * RAD_TO_DEG;Servo joint2;  // Elbow joint servo

  theta2_deg = theta2_rad * RAD_TO_DEG;

  // Pin definitions for 2-DOF robotic arm

  while (theta1_deg < 0) theta1_deg += 360;const int JOINT1_PIN = 9;   // Base joint (shoulder) connected to pin 9

  while (theta1_deg >= 360) theta1_deg -= 360;const int JOINT2_PIN = 10;  // Elbow joint connected to pin 10

  

  return true;// Servo angle constraints (in degrees)

}const int JOINT1_MIN = 0;    // Minimum angle for joint 1

const int JOINT1_MAX = 180;  // Maximum angle for joint 1

int kinematicToServoAngle(float kinematic_angle, int joint_number) {const int JOINT2_MIN = 0;    // Minimum angle for joint 2

  int servo_angle;const int JOINT2_MAX = 180;  // Maximum angle for joint 2

  

  if (joint_number == 1) {// Servo offset calibration (adjust for your specific servos)

    servo_angle = (int)(kinematic_angle * 0.5) + 90;const int JOINT1_OFFSET = 0;   // Offset to center joint 1 at 0 degrees (changed from 90)

  } else {const int JOINT2_OFFSET = 0;   // Offset to center joint 2 at 0 degrees (changed from 90)

    servo_angle = (int)(kinematic_angle) + 90;

  }// Servo control parameters

  const int SERVO1_CENTER = 90;  // Center position for servo 1 (stops continuous rotation)

  return constrain(servo_angle, 0, 180);const int SERVO2_CENTER = 90;  // Center position for servo 2

}

// ========================= KINEMATIC VARIABLES =========================

bool moveToPosition(float x, float y) {// Current joint angles (in degrees)

  Serial.printf("Moving to: (%.2f, %.2f)\n", x, y);float theta1 = 0.0;  // Joint 1 angle (base/shoulder)

  float theta2 = 0.0;  // Joint 2 angle (elbow)

  float new_theta1, new_theta2;

  if (!calculateInverseKinematics(x, y, new_theta1, new_theta2)) {// Target position variables

    return false;float target_x = 0.0;  // Target X coordinate

  }float target_y = 0.0;  // Target Y coordinate

  

  Serial.printf("Joint angles: θ1=%.2f°, θ2=%.2f°\n", new_theta1, new_theta2);// Workspace limits

  const float MAX_REACH = L1 + L2;  // Maximum reachable distance

  int servo1_angle = kinematicToServoAngle(new_theta1, 1);const float MIN_REACH = abs(L1 - L2);  // Minimum reachable distance

  int servo2_angle = kinematicToServoAngle(new_theta2, 2);

  // Solution selection preference

  Serial.printf("Servo commands: S1=%d°, S2=%d°\n", servo1_angle, servo2_angle);enum SolutionType {

    ELBOW_UP,    // Prefer elbow-up configuration

  joint1.write(servo1_angle);  ELBOW_DOWN,  // Prefer elbow-down configuration

  joint2.write(servo2_angle);  CLOSEST      // Choose solution closest to current position

  };

  theta1 = new_theta1;SolutionType preferred_solution = ELBOW_UP;

  theta2 = new_theta2;

  // ========================= MATHEMATICAL CONSTANTS =========================

  Serial.println("Movement complete!");// Using ESP32 built-in mathematical constants

  return true;// PI, DEG_TO_RAD, and RAD_TO_DEG are already defined in Arduino.h

}const float EPSILON = 0.001;  // Small value for numerical comparisons



void setup() {// ========================= INVERSE KINEMATICS FUNCTIONS =========================

  Serial.begin(115200);

  Serial.println("ESP32-S3 Inverse Kinematics Controller");/*

  Serial.println("Commands:"); * INVERSE KINEMATICS MATHEMATICAL MODEL:

  Serial.println("  x,y        -> Move to coordinates (example: 12.5,8.3)"); * 

  Serial.println("  S1,degrees -> Move servo 1 (example: S1,90)"); * Given target position (x, y), calculate joint angles (θ1, θ2)

  Serial.println("  S2,degrees -> Move servo 2 (example: S2,45)"); * 

   * Equations:

  joint1.attach(JOINT1_PIN); * x = L1*cos(θ1) + L2*cos(θ1 + θ2)

  joint2.attach(JOINT2_PIN); * y = L1*sin(θ1) + L2*sin(θ1 + θ2)

   * 

  joint1.write(90); * Solution using Law of Cosines:

  joint2.write(90); * D = sqrt(x² + y²)  [Distance from origin to target]

  delay(1000); * 

   * cos(θ2) = (D² - L1² - L2²) / (2*L1*L2)

  Serial.println("Ready for commands!"); * θ2 = ±acos(cos(θ2))  [Two solutions: elbow up/down]

} * 

 * θ1 = atan2(y, x) - atan2(L2*sin(θ2), L1 + L2*cos(θ2))

void loop() { */

  if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');struct InverseKinematicsSolution {

    input.trim();  bool valid;           // Solution validity flag

      float theta1_deg;     // Joint 1 angle in degrees

    // Check for servo commands (S1,degrees or S2,degrees)  float theta2_deg;     // Joint 2 angle in degrees

    if (input.startsWith("S1,")) {  float error;          // Solution error metric

      int degrees = input.substring(3).toInt();  String config_type;   // "ELBOW_UP" or "ELBOW_DOWN"

      degrees = constrain(degrees, 0, 180);};

      joint1.write(degrees);

      Serial.printf("Servo 1 moved to: %d degrees\n", degrees);// Workspace validation function

    }bool isPositionReachable(float x, float y) {

    else if (input.startsWith("S2,")) {  float distance = sqrt(x*x + y*y);

      int degrees = input.substring(3).toInt();  

      degrees = constrain(degrees, 0, 180);  // Check if position is within workspace boundaries

      joint2.write(degrees);  if (distance > MAX_REACH + EPSILON) {

      Serial.printf("Servo 2 moved to: %d degrees\n", degrees);    Serial.println("❌ Position beyond maximum reach");

    }    return false;

    // Check for coordinate commands (x,y)  }

    else {  

      int comma = input.indexOf(',');  if (distance < MIN_REACH - EPSILON) {

      if (comma > 0) {    Serial.println("❌ Position too close (below minimum reach)");

        float x = input.substring(0, comma).toFloat();    return false;

        float y = input.substring(comma + 1).toFloat();  }

        moveToPosition(x, y);  

      } else {  // Check for singularities (near-zero configurations)

        Serial.println("Invalid format!");  if (distance < EPSILON) {

        Serial.println("Use: x,y (coordinates) or S1,degrees or S2,degrees");    Serial.println("❌ Position at origin (singularity)");

        Serial.println("Examples: 12.5,8.3 or S1,90 or S2,45");    return false;

      }  }

    }  

  }  return true;

}}

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