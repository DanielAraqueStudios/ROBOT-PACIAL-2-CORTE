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

// ✅ NUEVO: Límites físicos de los servos
const int SERVO1_MIN = 0;   // Servo 1: 0-180°
const int SERVO1_MAX = 180;
const int SERVO2_MIN = 0;   // Servo 2: FULL RANGE 0-180° (NOT LIMITED)
const int SERVO2_MAX = 180;

float theta1 = 0.0;
float theta2 = 0.0;

bool isPositionReachable(float x, float y) {
  float distance = sqrt(x*x + y*y);
  // ✅ FIXED: Allow full range from 0.0 to 20.0 cm (including origin)
  return (distance <= MAX_REACH && distance >= MIN_REACH);
}

bool calculateInverseKinematics(float target_x, float target_y, float &theta1_deg, float &theta2_deg) {
  if (!isPositionReachable(target_x, target_y)) {
    Serial.println("Position not reachable!");
    return false;
  }
  
  // ✅ NUEVO: Validación de cuadrantes permitidos (solo Y >= 0)
  if (target_y < 0) {
    Serial.println("Negative Y coordinates not allowed! Only X,Y and -X,Y quadrants supported.");
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
  
  // ✅ CORREGIDO: Manejo correcto de ángulos negativos para θ1
  // Convertir a rango 0-360° y luego limitar a 0-180° para servo
  while (theta1_deg < 0) theta1_deg += 360;
  while (theta1_deg >= 360) theta1_deg -= 360;
  
  // Si el ángulo está en el rango 180-360°, necesitamos la configuración alternativa
  if (theta1_deg > 180) {
    theta1_deg = 360 - theta1_deg;  // Convertir a ángulo equivalente en 0-180°
  }
  
  // ✅ NUEVO: Validar que el ángulo resultante esté en rango válido para servo
  if (theta1_deg < 0 || theta1_deg > 180) {
    Serial.println("Joint 1 angle out of valid range (0-180°)!");
    return false;
  }
  
  return true;
}

int kinematicToServoAngle(float kinematic_angle, int joint_number) {
  int servo_angle;
  
  if (joint_number == 1) {
    // Joint 1 (Base/Shoulder): CORREGIDO - Mapear 0-180° cinemático directamente a 0-180° servo
    // Para trabajar en cuadrantes I y II (Y >= 0), necesitamos mapeo directo
    servo_angle = (int)(kinematic_angle);
  } else {
    // Joint 2 (Elbow): CORREGIDO - Mapear θ2 (0-180°) a servo (0-180°) sin offset
    // Para θ2 = 0° → servo = 0° (brazo extendido)
    // Para θ2 = 90° → servo = 90° (brazo doblado 90°)
    // Para θ2 = 180° → servo = 180° (brazo completamente doblado)
    servo_angle = (int)(kinematic_angle);
  }
  
  // ✅ GARANTIZAR: Limitar estrictamente a rango 0-180° para ambos servos
  return constrain(servo_angle, 0, 180);
}

bool moveToPosition(float x, float y) {
  Serial.printf("🎯 Moving to: (%.2f, %.2f)\n", x, y);
  Serial.printf("Distance from origin: %.2f cm\n", sqrt(x*x + y*y));
  
  float new_theta1, new_theta2;
  if (!calculateInverseKinematics(x, y, new_theta1, new_theta2)) {
    return false;
  }
  
  Serial.printf("📐 Joint angles: θ1=%.2f°, θ2=%.2f°\n", new_theta1, new_theta2);
  
  int servo1_angle = kinematicToServoAngle(new_theta1, 1);
  int servo2_angle = kinematicToServoAngle(new_theta2, 2);
  
  Serial.printf("🔧 Servo commands: S1=%d°, S2=%d°\n", servo1_angle, servo2_angle);
  
  // Forward kinematics verification
  float verify_x = L1 * cos(new_theta1 * DEG_TO_RAD) + L2 * cos((new_theta1 + new_theta2) * DEG_TO_RAD);
  float verify_y = L1 * sin(new_theta1 * DEG_TO_RAD) + L2 * sin((new_theta1 + new_theta2) * DEG_TO_RAD);
  Serial.printf("✅ Verification: Calculated position (%.2f, %.2f)\n", verify_x, verify_y);
  
  joint1.write(servo1_angle);
  joint2.write(servo2_angle);
  
  theta1 = new_theta1;
  theta2 = new_theta2;
  
  Serial.println("✅ Movement complete!");
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("🤖 ESP32-S3 Inverse Kinematics Controller - FIXED VERSION");
  Serial.println("=== WORKSPACE LIMITATIONS ===");
  Serial.println("• Only works in X,Y and -X,Y planes (Y >= 0)");
  Serial.println("• Joint 1: 0-180° servo range (base rotation)");
  Serial.println("• Joint 2: 0-180° servo range (elbow movement)");
  Serial.println("• Maximum reach: 20.0 cm | Minimum reach: 0.0 cm");
  Serial.println("• Direct servo mapping: θ → servo (no offsets)");
  Serial.println("");
  Serial.println("Commands:");
  Serial.println("  x,y        -> Move to coordinates (example: 10,5)");
  Serial.println("  S1,degrees -> Move servo 1: 0-180° (example: S1,90)");
  Serial.println("  S2,degrees -> Move servo 2: 0-180° (example: S2,0)");
  Serial.println("");
  Serial.println("Valid test coordinates:");
  Serial.println("  10,5   (Quadrant I: +X,+Y)");
  Serial.println("  -10,5  (Quadrant II: -X,+Y)");
  Serial.println("  15,0   (Positive X axis)");
  Serial.println("  -15,0  (Negative X axis)");
  Serial.println("  0,18   (Maximum Y reach)");
  Serial.println("");
  Serial.println("✅ Initial Position: Joint1=90°, Joint2=0° (arm extended forward)");
  
  joint1.attach(JOINT1_PIN);
  joint2.attach(JOINT2_PIN);
  
  // ✅ CORREGIDO: Posiciones iniciales para el nuevo mapeo
  joint1.write(90);  // θ1 = 90° → servo = 90° (pointing forward)
  joint2.write(0);   // θ2 = 0° → servo = 0° (arm extended)
  delay(1000);
  
  Serial.println("🚀 Ready for commands!");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check for servo commands (S1,degrees or S2,degrees)
    if (input.startsWith("S1,")) {
      int degrees = input.substring(3).toInt();
      degrees = constrain(degrees, SERVO1_MIN, SERVO1_MAX);  // 0-180°
      joint1.write(degrees);
      Serial.printf("Servo 1 moved to: %d degrees\n", degrees);
    }
    else if (input.startsWith("S2,")) {
      int degrees = input.substring(3).toInt();
      degrees = constrain(degrees, SERVO2_MIN, SERVO2_MAX);  // FULL 0-180° range
      joint2.write(degrees);
      Serial.printf("Servo 2 moved to: %d degrees (full 0-180° range)\n", degrees);
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