#include <ESP32Servo.h>
#include <math.h>

Servo joint1;
Servo joint2;

const int JOINT1_PIN = 9;
const int JOINT2_PIN = 10;

const float L1 = 10.0;
const float L2 = 10.0;
const float MAX_REACH = L1 + L2;
const float MIN_REACH = abs(L1 - L2);

float theta1 = 0.0;
float theta2 = 0.0;

bool isPositionReachable(float x, float y) {
  float distance = sqrt(x*x + y*y);
  return (distance <= MAX_REACH && distance >= MIN_REACH && distance > 0.001);
}

bool calculateInverseKinematics(float target_x, float target_y, float &theta1_deg, float &theta2_deg) {
  if (!isPositionReachable(target_x, target_y)) {
    Serial.println("Position not reachable!");
    return false;
  }
  
  float D = sqrt(target_x*target_x + target_y*target_y);
  float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  
  if (cos_theta2 < -1.0 || cos_theta2 > 1.0) {
    Serial.println("No solution exists!");
    return false;
  }
  
  float theta2_rad = acos(cos_theta2);
  float alpha = atan2(target_y, target_x);
  float beta = atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));
  float theta1_rad = alpha - beta;
  
  theta1_deg = theta1_rad * RAD_TO_DEG;
  theta2_deg = theta2_rad * RAD_TO_DEG;
  
  while (theta1_deg < 0) theta1_deg += 360;
  while (theta1_deg >= 360) theta1_deg -= 360;
  
  return true;
}

int kinematicToServoAngle(float kinematic_angle, int joint_number) {
  int servo_angle;
  
  if (joint_number == 1) {
    servo_angle = (int)(kinematic_angle * 0.5) + 90;
  } else {
    servo_angle = (int)(kinematic_angle) + 90;
  }
  
  return constrain(servo_angle, 0, 180);
}

bool moveToPosition(float x, float y) {
  Serial.printf("Moving to: (%.2f, %.2f)\n", x, y);
  
  float new_theta1, new_theta2;
  if (!calculateInverseKinematics(x, y, new_theta1, new_theta2)) {
    return false;
  }
  
  Serial.printf("Joint angles: θ1=%.2f°, θ2=%.2f°\n", new_theta1, new_theta2);
  
  int servo1_angle = kinematicToServoAngle(new_theta1, 1);
  int servo2_angle = kinematicToServoAngle(new_theta2, 2);
  
  Serial.printf("Servo commands: S1=%d°, S2=%d°\n", servo1_angle, servo2_angle);
  
  joint1.write(servo1_angle);
  joint2.write(servo2_angle);
  
  theta1 = new_theta1;
  theta2 = new_theta2;
  
  Serial.println("Movement complete!");
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 Inverse Kinematics Controller");
  Serial.println("Commands:");
  Serial.println("  x,y        -> Move to coordinates (example: 12.5,8.3)");
  Serial.println("  S1,degrees -> Move servo 1 (example: S1,90)");
  Serial.println("  S2,degrees -> Move servo 2 (example: S2,45)");
  
  joint1.attach(JOINT1_PIN);
  joint2.attach(JOINT2_PIN);
  
  joint1.write(90);
  joint2.write(90);
  delay(1000);
  
  Serial.println("Ready for commands!");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check for servo commands (S1,degrees or S2,degrees)
    if (input.startsWith("S1,")) {
      int degrees = input.substring(3).toInt();
      degrees = constrain(degrees, 0, 180);
      joint1.write(degrees);
      Serial.printf("Servo 1 moved to: %d degrees\n", degrees);
    }
    else if (input.startsWith("S2,")) {
      int degrees = input.substring(3).toInt();
      degrees = constrain(degrees, 0, 180);
      joint2.write(degrees);
      Serial.printf("Servo 2 moved to: %d degrees\n", degrees);
    }
    // Check for coordinate commands (x,y)
    else {
      int comma = input.indexOf(',');
      if (comma > 0) {
        float x = input.substring(0, comma).toFloat();
        float y = input.substring(comma + 1).toFloat();
        moveToPosition(x, y);
      } else {
        Serial.println("Invalid format!");
        Serial.println("Use: x,y (coordinates) or S1,degrees or S2,degrees");
        Serial.println("Examples: 12.5,8.3 or S1,90 or S2,45");
      }
    }
  }
}