#include <ESP32Servo.h>
#include <math.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

Servo joint1;
Servo joint2;

// ✅ CORREGIDO: Pines 13 y 12 para servos (como estaba antes)
const int JOINT1_PIN = 13;   // Pin para servo 1 
const int JOINT2_PIN = 12;   // Pin para servo 2

// ✅ NUEVO: Configuración del teclado matricial 4x4
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// ✅ CORREGIDO: Pines correctos del teclado según README
byte rowPins[ROWS] = {21, 19, 18, 5}; // Filas: corregido según pinout
byte colPins[COLS] = {17, 16, 4, 0}; // Columnas: corregido según pinout
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ✅ CORREGIDO: Configuración del LCD I2C (como estaba antes)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección I2C, 16 columnas, 2 filas
// SDA: Pin 8, SCL: Pin 9 (definido por hardware I2C del ESP32-S3)

// ✅ CORREGIDO: Variables para modo hardware
enum HardwareMode {
  MODE_IDLE,
  MODE_ANGLE_INPUT,    // Tecla A: Ingresar ángulos
  MODE_POSITION_INPUT  // Tecla B: Ingresar posición
};

HardwareMode hardwareMode = MODE_IDLE;  // Variable de estado del modo
String currentInput = "";               // Buffer de entrada actual
int inputStep = 0;                     // Paso actual en la entrada (0 o 1)


float input_theta1 = 90.0;
float input_theta2 = 90.0;
float input_x = 10.0;
float input_y = 10.0;

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

// ✅ NUEVO: Funciones para modo hardware
void calculateForwardKinematics(float theta1_deg, float theta2_deg, float &x, float &y) {
  /**
   * Calcula la posición X,Y a partir de los ángulos de las articulaciones
   * Usado en MODO A (ingresar ángulos → ver posición)
   */
  float theta1_rad = theta1_deg * DEG_TO_RAD;
  float theta2_rad = theta2_deg * DEG_TO_RAD;
  
  x = L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad);
  y = L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad);
}

void updateLCD() {
  /**
   * Actualiza la pantalla LCD con el formato requerido:
   * Línea 1: Ángulos (θ1, θ2)
   * Línea 2: Posición (X, Y)
   */
  lcd.clear();
  
  // Primera fila: Ángulos
  lcd.setCursor(0, 0);
  lcd.print("A1:");
  lcd.print((int)input_theta1);
  lcd.print(" A2:");
  lcd.print((int)input_theta2);
  
  // Segunda fila: Posición
  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(input_x, 1);
  lcd.print(" Y:");
  lcd.print(input_y, 1);
}

void handleModeA() {
  /**
   * MODO A: Ingresar ángulos directos (0-180°)
   * - Usuario ingresa θ1 y θ2
   * - Sistema calcula y muestra posición X,Y resultante
   */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MODO A: ANGULOS");
  lcd.setCursor(0, 1);
  
  if (inputStep == 0) {
    lcd.print("Ingrese Ang1:");
    lcd.print(currentInput);
  } else {
    lcd.print("Ingrese Ang2:");
    lcd.print(currentInput);
  }
}

void handleModeB() {
  /**
   * MODO B: Ingresar posición X,Y
   * - Usuario ingresa coordenadas X,Y
   * - Sistema calcula y muestra ángulos θ1,θ2 resultantes
   */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MODO B: POSICION");
  lcd.setCursor(0, 1);
  
  if (inputStep == 0) {
    lcd.print("Ingrese X:");
    lcd.print(currentInput);
  } else {
    lcd.print("Ingrese Y:");
    lcd.print(currentInput);
  }
}

void processAngleInput(char key) {
  /**
   * Procesa la entrada de ángulos en MODO A
   */
  if (key >= '0' && key <= '9') {
    currentInput += key;
    handleModeA();
  }
  else if (key == '#') {
    if (inputStep == 0) {
      // Confirmar θ1
      input_theta1 = constrain(currentInput.toFloat(), 0, 180);
      currentInput = "";
      inputStep = 1;
      handleModeA();
    } else {
      // Confirmar θ2 y calcular posición
      input_theta2 = constrain(currentInput.toFloat(), 0, 180);
      
      // Calcular posición resultante
      calculateForwardKinematics(input_theta1, input_theta2, input_x, input_y);
      
      currentInput = "";
      inputStep = 0;
      updateLCD();
    }
  }
  else if (key == '*') {
    // Borrar último carácter
    if (currentInput.length() > 0) {
      currentInput.remove(currentInput.length() - 1);
      if (inputStep == 0) handleModeA();
      else handleModeA();
    }
  }
  else if (key == 'C') {
    executeMovement();
  }
}

void processPositionInput(char key) {
  /**
   * Procesa la entrada de posición en MODO B
   */
  if ((key >= '0' && key <= '9') || key == '.') {
    currentInput += key;
    handleModeB();
  }
  else if (key == '#') {
    if (inputStep == 0) {
      // Confirmar X
      input_x = currentInput.toFloat();
      currentInput = "";
      inputStep = 1;
      handleModeB();
    } else {
      // Confirmar Y y calcular ángulos
      input_y = currentInput.toFloat();
      
      // Calcular ángulos resultantes
      if (calculateInverseKinematics(input_x, input_y, input_theta1, input_theta2)) {
        Serial.printf("✅ Posición válida: (%.1f, %.1f)\n", input_x, input_y);
      } else {
        Serial.printf("❌ Posición inválida: (%.1f, %.1f)\n", input_x, input_y);
        // Mantener valores anteriores si la posición no es válida
        lcd.setCursor(0, 1);
        lcd.print("POSICION INVALIDA");
        delay(2000);
      }
      
      currentInput = "";
      inputStep = 0;
      updateLCD();
    }
  }
  else if (key == '*') {
    // Borrar último carácter
    if (currentInput.length() > 0) {
      currentInput.remove(currentInput.length() - 1);
      if (inputStep == 0) handleModeB();
      else handleModeB();
    }
  }
  else if (key == 'C') {
    executeMovement();
  }
}

void executeMovement() {
  /**
   * Ejecuta el movimiento con los valores actuales
   * Se llama al presionar tecla C
   */
  Serial.printf("🎯 Ejecutando: θ1=%.0f°, θ2=%.0f° → X=%.1f, Y=%.1f\n", 
                input_theta1, input_theta2, input_x, input_y);
  
  // Convertir a comandos de servo y mover
  int servo1_angle = kinematicToServoAngle(input_theta1, 1);
  int servo2_angle = kinematicToServoAngle(input_theta2, 2);
  
  joint1.write(servo1_angle);
  joint2.write(servo2_angle);
  
  // Actualizar variables globales
  theta1 = input_theta1;
  theta2 = input_theta2;
  
  // Mostrar confirmación en LCD
  lcd.setCursor(0, 1);
  lcd.print("EJECUTADO!      ");
  delay(1500);
  updateLCD();
}

void setup() {
  Serial.begin(115200);
  Serial.println("🤖 ESP32-S3 Inverse Kinematics Controller - HARDWARE MODE");
  Serial.println("=== DUAL MODE OPERATION ===");
  Serial.println("• SERIAL MODE: Send commands via Serial Monitor");
  Serial.println("• HARDWARE MODE: Use keypad + LCD for local control");
  Serial.println("");
  Serial.println("HARDWARE MODE Controls:");
  Serial.println("• Tecla A: Modo Ángulos (ingresar θ1, θ2)");
  Serial.println("• Tecla B: Modo Posición (ingresar X, Y)");
  Serial.println("• Tecla C: Ejecutar movimiento");
  Serial.println("• Números: Ingresar valores");
  Serial.println("• #: Confirmar entrada");
  Serial.println("• *: Borrar último dígito");
  Serial.println("");
  Serial.println("SERIAL MODE Commands:");
  Serial.println("  x,y        -> Move to coordinates (example: 10,5)");
  Serial.println("  S1,degrees -> Move servo 1: 0-180° (example: S1,90)");
  Serial.println("  S2,degrees -> Move servo 2: 0-180° (example: S2,0)");
  Serial.println("");
  Serial.println("✅ Workspace: 0-20cm reach, Y >= 0 only");
  
  // ✅ NUEVO: Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ROBOT 2-DOF");
  lcd.setCursor(0, 1);
  lcd.print("A:ANG B:POS C:GO");
  
  // ✅ Inicializar servos
  joint1.attach(JOINT1_PIN);
  joint2.attach(JOINT2_PIN);
  
  // ✅ Posición inicial
  joint1.write(90);  // θ1 = 90° → servo = 90° (pointing forward)
  joint2.write(0);   // θ2 = 0° → servo = 0° (arm extended)
  delay(1000);
  
  // ✅ Inicializar valores por defecto para modo hardware
  input_theta1 = 90.0;
  input_theta2 = 0.0;
  calculateForwardKinematics(input_theta1, input_theta2, input_x, input_y);
  
  Serial.println("🚀 Ready! Use Serial commands or Hardware keypad");
}



void loop() {
  // Handle hardware keypad input
  char key = keypad.getKey();
  if (key) {
    Serial.print("Key pressed: ");
    Serial.println(key);
    
    // Handle mode selection
    if (key == 'A') {
      hardwareMode = MODE_ANGLE_INPUT;
      currentInput = "";
      inputStep = 0;
      Serial.println("Hardware Mode A: Angle Input");
      updateLCD();
    }
    else if (key == 'B') {
      hardwareMode = MODE_POSITION_INPUT;
      currentInput = "";
      inputStep = 0;
      Serial.println("Hardware Mode B: Position Input");
      updateLCD();
    }
    else {
      // Process input based on current mode
      switch(hardwareMode) {
        case MODE_ANGLE_INPUT:
          processAngleInput(key);
          break;
        case MODE_POSITION_INPUT:
          processPositionInput(key);
          break;
        case MODE_IDLE:
          lcd.setCursor(0, 1);
          lcd.print("Press A or B    ");
          break;
      }
    }
  }

  // Handle serial commands
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