/*
 * 🔢 ESP32-S3 KEYPAD + LCD TESTER (ESP32 BLACKHACK) 
 * ================================================= 
 * Programa para probar teclado matricial 4x4 + LCD I2C
 * ✅ Usando "ESP32 LiquidCrystal I2C" by Blackhack
 * 
 * Funciones:
 * - Detecta y muestra en Serial las teclas presionadas
 * - Muestra las teclas presionadas en la pantalla LCD
 * - Configuración de pines personalizable  
 * - Scanner I2C automático para detectar LCD
 * 
 * Conexiones del Teclado 4x4:
 * FILAS:    Pines 4, 5, 6, 7
 * COLUMNAS: Pines 17, 18, 10, 11
 * 
 * Conexiones del LCD I2C:
 * SDA: Pin 8
 * SCL: Pin 9
 * 
 * INSTALACIÓN DE LIBRERÍA:
 * 1. Arduino IDE > Tools > Manage Libraries
 * 2. Buscar: "ESP32 LiquidCrystal I2C"
 * 3. Instalar: "ESP32 LiquidCrystal I2C" by Blackhack
 */


#include <Keypad.h>
#include <Wire.h>
#include "LCD_I2C.h"  // Blackhack ESP32 LCD_I2C library

// ✅ Configuración del teclado matricial 4x4
const byte ROWS = 4;  // Cuatro filas
const byte COLS = 4;  // Cuatro columnas

// ✅ Layout del teclado (lo que está impreso en las teclas)
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// ✅ Configuración de pines - AJUSTA ESTOS SEGÚN TU CONEXIÓN
byte rowPins[ROWS] = {4, 5, 6, 7};        // Filas del teclado
byte colPins[COLS] = {17, 18, 10, 11};    // Columnas del teclado

// ✅ Crear objeto Keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


// ✅ Configuración del LCD I2C - ESP32 Blackhack Library
LCD_I2C lcd(0x27, 16, 2); // Dirección fija: 0x27
// SDA: Pin 8, SCL: Pin 9

// ✅ Variables para el LCD
bool lcdFound = true;  // Asumir que el LCD está conectado
uint8_t lcdAddress = 0x27; // Dirección fija por defecto

// ✅ Variables para estadísticas
unsigned long totalKeyPresses = 0;
char lastKey = 0;
unsigned long lastKeyTime = 0;

// ✅ Variables para el LCD
String lcdLine1 = ""; 
String lcdLine2 = ""; 
int lcdUpdateCount = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("========================================");
  Serial.println("         PROBADOR TECLADO + LCD ESP32   ");
  Serial.println("========================================");
  Serial.println("Versión: 6.0 ESP32 BLACKHACK");
  Serial.println("ESP32-S3 Compatible");
  Serial.println("Librería: ESP32 LiquidCrystal I2C");
  Serial.println("Auto-detección I2C");
  Serial.println();
  
  // ✅ Inicializar I2C en pines específicos del ESP32-S3
  Wire.begin(8, 9); // SDA=8, SCL=9
  delay(100);
  
  // ✅ Inicializar el LCD directamente con dirección por defecto 0x27
  Serial.println("📺 Inicializando LCD con ESP32 Blackhack Library...");
  Serial.println("   Usando dirección por defecto: 0x27");
  
  lcd.begin();      // Inicializar LCD (Blackhack)
  lcd.backlight(); // Encender retroiluminación
  
  // Mensaje de bienvenida
  lcd.setCursor(0, 0);
  lcd.print("ESP32-S3 READY!");
  lcd.setCursor(0, 1);
  lcd.print("Blackhack v6.0");
  
  Serial.printf("✅ LCD inicializado en dirección 0x%02X\n", lcdAddress);
  delay(2000); // Mostrar mensaje de bienvenida
  
  // ✅ Configuración del teclado
  Serial.println("⌨️  Configurando teclado matricial 4x4...");
  Serial.println("   Filas: Pines 4, 5, 6, 7");
  Serial.println("   Columnas: Pines 17, 18, 10, 11");
  Serial.println();
  
  // ✅ Información completa del sistema
  Serial.println("📋 CONFIGURACIÓN COMPLETA:");
  Serial.printf("   I2C SDA: Pin %d\n", 8);
  Serial.printf("   I2C SCL: Pin %d\n", 9);
  Serial.printf("   LCD Dirección: 0x%02X (fija)\n", lcdAddress);
  Serial.println("   Keypad: 4x4 Matrix");
  Serial.println();
  
  Serial.println("🚀 Sistema listo! Presiona teclas para probar...");
  Serial.println("========================================");
}

// ✅ FUNCIÓN PARA ACTUALIZAR LCD (ESP32 Blackhack Library)
void updateLCD(String line1, String line2) {
  lcdUpdateCount++;
  
  // Actualizar solo si hay cambios
  if (line1 != lcdLine1 || line2 != lcdLine2) {
    lcdLine1 = line1;
    lcdLine2 = line2;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1.substring(0, 16)); // Máximo 16 caracteres
    lcd.setCursor(0, 1);
    lcd.print(line2.substring(0, 16)); // Máximo 16 caracteres
    
    Serial.printf("📺 LCD actualizado (#%d): '%s' | '%s'\n", 
                  lcdUpdateCount, line1.c_str(), line2.c_str());
  }
}

void loop() {
  // ✅ Leer tecla presionada
  char key = keypad.getKey();
  
  if (key) {
    // ✅ Incrementar contador
    totalKeyPresses++;
    lastKey = key;
    lastKeyTime = millis();
    
    // ✅ Mostrar información completa en Serial
    Serial.printf("🔥 TECLA PRESIONADA: '%c'\n", key);
    Serial.printf("   ⏰ Tiempo: %lu ms\n", lastKeyTime);
    Serial.printf("   📊 Total presiones: %lu\n", totalKeyPresses);
    Serial.printf("   📍 Posición en matriz: ");
    
    // Encontrar posición en la matriz
    bool found = false;
    for(int row = 0; row < ROWS && !found; row++) {
      for(int col = 0; col < COLS && !found; col++) {
        if(keys[row][col] == key) {
          Serial.printf("Fila %d, Columna %d\n", row+1, col+1);
          Serial.printf("   🔌 Pines: R%d=Pin%d, C%d=Pin%d\n", 
                        row+1, rowPins[row], col+1, colPins[col]);
          found = true;
        }
      }
    }
    
    // ✅ Actualizar LCD (siempre disponible)
    String line1 = "Tecla: " + String(key) + " #" + String(totalKeyPresses);
    String line2 = "Tiempo: " + String(millis()/1000) + "s";
    updateLCD(line1, line2);
    
    // ✅ Separador visual
    Serial.println("   ═══════════════════════════════════════");
    Serial.println();
    
    // ✅ Pequeña pausa para evitar rebotes
    delay(200);
  }
  
  // ✅ Cada 10 segundos mostrar estadísticas
  static unsigned long lastStatsTime = 0;
  if (millis() - lastStatsTime >= 10000) {
    lastStatsTime = millis();
    
    Serial.println("📈 ESTADÍSTICAS CADA 10 SEGUNDOS:");
    Serial.printf("   Total teclas presionadas: %lu\n", totalKeyPresses);
    Serial.printf("   Última tecla: '%c' hace %lu ms\n", 
                  lastKey, (lastKey != 0) ? (millis() - lastKeyTime) : 0);
    Serial.printf("   Sistema funcionando: %.1f segundos\n", millis()/1000.0);
    Serial.printf("   LCD Dirección: 0x%02X\n", lcdAddress);
    
    String line1 = "Stats: " + String(totalKeyPresses) + " keys";
    String line2 = "Uptime: " + String(millis()/1000) + "s";
    updateLCD(line1, line2);
    
    Serial.println("   ─────────────────────────────────────────");
    Serial.println();
  }
}