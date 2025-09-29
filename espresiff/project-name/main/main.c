#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp32/rom/ets_sys.h"
#include "hal/gpio_types.h"

#define pi 3.141592

// Definiciones para servomotores
#define SERVO1_PIN 12        // Pin para servo 1 (ángulo a1)
#define SERVO2_PIN 14        // Pin para servo 2 (ángulo a2)
#define SERVO_FREQ 50        // Frecuencia PWM para servos (50Hz)
#define SERVO_RESOLUTION LEDC_TIMER_16_BIT
#define SERVO_MIN_DUTY 1638  // ~1ms pulse (0°)
#define SERVO_MAX_DUTY 8192  // ~2ms pulse (180°)

char n[4] = {0}; //velocidad
int nd = 0; //posición vector velocidad
int N=0; //valor velocidad
int C=0;
bool A = false;
bool B = false;
int val[2] = {0,0};
int val2[2]={0,0};
int i=0;
int L1 = 2;
int L2 = 2;

// Variables globales para ángulos de servos
float servo1_angle = 0;  // ángulo a1
float servo2_angle = 0;  // ángulo a2

// Función para convertir ángulo a duty cycle
uint32_t angle_to_duty(float angle) {
    // Limitar ángulo entre 0 y 180 grados
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Conversión lineal: 0° = SERVO_MIN_DUTY, 180° = SERVO_MAX_DUTY
    return SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY);
}

// Función para mover los servos
void move_servos(float angle1, float angle2) {
    uint32_t duty1 = angle_to_duty(angle1);
    uint32_t duty2 = angle_to_duty(angle2);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    
    printf("Servo 1: %.1f° (duty: %d), Servo 2: %.1f° (duty: %d)\n", 
           angle1, duty1, angle2, duty2);
}

// Configuración de servomotores
void servo_init(void) {
    // Configurar timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
    
    // Configurar canal 0 para servo 1
    ledc_channel_config_t servo1_config = {
        .gpio_num = SERVO1_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = SERVO_MIN_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&servo1_config);
    
    // Configurar canal 1 para servo 2
    ledc_channel_config_t servo2_config = {
        .gpio_num = SERVO2_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = SERVO_MIN_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&servo2_config);
    
    printf("Servomotores inicializados en pines %d y %d\n", SERVO1_PIN, SERVO2_PIN);
    
    // Posición inicial (0 grados)
    move_servos(0, 0);
}

void pins(void)
{
    gpio_reset_pin(2);
    gpio_reset_pin(13);
    gpio_reset_pin(21);
    
    // Teclado - Filas (salida)
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_direction(19, GPIO_MODE_OUTPUT);
    gpio_set_direction(18, GPIO_MODE_OUTPUT);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);

    // Teclado - Columnas (entrada con pull-down)
    gpio_set_direction(17, GPIO_MODE_INPUT);
    gpio_set_direction(16, GPIO_MODE_INPUT);
    gpio_set_direction(4, GPIO_MODE_INPUT);
    gpio_set_direction(2, GPIO_MODE_INPUT);
    gpio_pulldown_en(17);
    gpio_pulldown_en(16);
    gpio_pulldown_en(4);
    gpio_pulldown_en(2);
}

char keya()
{
    char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    
    int row[4] = {21, 19, 18, 5};
    int col[4] = {17, 16, 4, 2};
    
    for(int i = 0; i < 4; i++)
    {
        gpio_set_level(row[i], 1);
        vTaskDelay(1);
        
        for(int j = 0; j < 4; j++)
        {
            if (gpio_get_level(col[j]) == 1) 
            {
                gpio_set_level(row[i], 0);
                vTaskDelay(50); 
                return keys[i][j];
            }
        }
        gpio_set_level(row[i], 0);
    }
    return 0;
}

void keyb(char key)
{
    if (key >= '0'  && key <= '9') {
        //  lee el vector n y almacena en cada posición 1 dato
        if (nd < 3) {
            n[nd++] = key;
            printf("Valor %s\n", n);
        }
    }
    if (key == 'A') {
        printf("modo A activado. \n");
        A = true;
        B = false;
    }
        
    if (key == 'B') 
    {
        printf("modo B activado ");
        A = false;
        B = true;
    }

    if(A==true && B==false)
    {
        if (key == 'C' && N >= 0 && N <= 180){
            int N = atoi(n);
            if(i < 2){
                val[i]=N;
                i++;
            }
            nd=0;
            memset(n, 0, sizeof(n));
            if(i>=2){
                i=0;
            }
        }
        
        if(val[0] != 0 && val[1] != 0){
            printf("Ang. 1: %d\n",val[0]);
            printf("Ang. 2: %d\n",val[1]);    
            
            float a1 = val[0]*pi/180.0f;    
            float a2 = val[1]*pi/180.0f;
            
            printf("Ang. 1: %f radianes\n",a1);
            printf("Ang. 2: %f radianes\n",a2);
            
            float px = L1*cos(a1)+ L2*cos(a1+a2);
            float py = L1*sin(a1)+L2*sin(a1+a2);
            
            printf("Px: %f\n",px);
            printf("Py: %f\n",py);
            
            // **MOVER SERVOS EN MODO A (cinemática directa)**
            servo1_angle = val[0];  // a1 en grados
            servo2_angle = val[1];  // a2 en grados
            move_servos(servo1_angle, servo2_angle);
        }
    }
    
    if(A==false && B==true)
    {
        if (key == 'C')
        {
            int N = atoi(n);
            
            if(i < 2){
                val2[i]=N;
                i++;
            }
            nd=0;
            memset(n, 0, sizeof(n));
            if(i>=2){
                i=0;
            }
        }
        if(key=='*'){
            val2[0]=-val2[0];
        }
        if(key=='#'){
            val2[1]=-val2[1];
        }
        
        printf("Px: %d\n",val2[0]);
        printf("Py: %d\n",val2[1]);    
        
        if(val2[0] != 0 || val2[1] != 0)
        {
            float s;
            float a1;
            float B;
            float a2;
            float h = sqrt((val2[0]*val2[0])+(val2[1]*val2[1]));
            
            // Verificar alcance
            if(h > (L1 + L2)) {
                printf("ERROR: Punto fuera de alcance\n");
                return;
            }
            
            B = acos(((L2*L2)-(h*h)-(L1*L1))/((-2)*h*L1)); //angulo beta
            
            if(val2[0] == 0) 
            {
                // Px = 0, usar ángulo directo
                s = pi/2;
                B=0;
                a2=90;  // 90 grados
            } else {
                s = atan(val2[1]/val2[0]); // atan maneja el cálculo básico
            }
            
            if(s < -(pi/2) - 0.000001 || s > (pi/2) + 0.000001 || B < 0 || B > 1){
                printf("error en cálculo\n");
            }
            else {                
                if(val2[0] == -(L1+L2) || val2[1] < 0){
                    a1 = 180;
                } else {
                    a1 = fabs((s-B)*180/pi); //angulo 1
                }
                    
                float n = h*sin(B);
                
                a2 = asin(n/L2)*180/pi;
                
                printf("hipotenusa %f\n",h);
                printf("beta %f\n",B);
                printf("sigma %f\n",s);
                printf("ang. 1 %f\n",a1);
                printf("n %f\n",n);
                printf("ang. 2 %f\n",a2);

                // **MOVER SERVOS EN MODO B (cinemática inversa)**
                servo1_angle = a1;
                servo2_angle = a2;
                move_servos(servo1_angle, servo2_angle);
            }         
        }
    }
}

void app_main()
{
    // Inicializar pines del teclado
    pins();
    
    // **INICIALIZAR SERVOMOTORES**
    servo_init();
    
    printf("Sistema iniciado. Servos en pines %d y %d\n", SERVO1_PIN, SERVO2_PIN);
    printf("Modo A: Ingresa ángulos directamente\n");
    printf("Modo B: Ingresa coordenadas Px, Py\n");
    
    while(1)
    {
        char key = keya();
        if (key != 0) {
            printf("Tecla: %c\n", key);
            keyb(key);
        }
        vTaskDelay(10); // Pequeño delay para estabilidad
    }
}