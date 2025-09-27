#include <ESP32Servo.h>

Servo joint1;
Servo joint2;

const int JOINT1_PIN = 9;
const int JOINT2_PIN = 10;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 Servo Test Starting...");
  
  joint1.attach(JOINT1_PIN);
  joint2.attach(JOINT2_PIN);
  
  Serial.println("Servos attached - Starting test sequence...");
  delay(1000);
}

void loop() {
  Serial.println("Moving Joint 1 to 90 degrees...");
  joint1.write(90);
  delay(2000);
  
  Serial.println("Moving Joint 2 to 90 degrees...");
  joint2.write(90);
  delay(2000);
  
  Serial.println("Moving Joint 1 to 0 degrees...");
  joint1.write(0);
  delay(2000);
  
  Serial.println("Moving Joint 2 to 0 degrees...");
  joint2.write(0);
  delay(2000);
  
  Serial.println("Cycle complete - Starting again...\n");
}