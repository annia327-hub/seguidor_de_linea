#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"

//REVISAR CÓDIGO

/*
#include "IRremoteESP8266.h"
#include "IRrecv.h"
#include "IRutils.h"
*/

// ===== DEFINICIÓN DE PINES =====
#define PIN_COM       1
#define PIN_S0        0
#define PIN_S1        2
#define PIN_S2        3
#define PIN_S3        4
#define PIN_DIR_IZQ   20
#define PIN_PWM_I     21
#define PIN_DIR_DER   6
#define PIN_PWM_D     5
#define PIN_LED       8
#define PIN_IR        10

// ===== CONFIGURACIÓN LEDC (PWM) =====
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES          LEDC_TIMER_8_BIT // 8 bits (0-255)
#define LEDC_FREQUENCY_PWM     20000            // 20 kHz para motores
#define LEDC_FREQUENCY_LED     5000             // 5 kHz para indicador

#define CHANNEL_PWM_I          LEDC_CHANNEL_0
#define CHANNEL_PWM_D          LEDC_CHANNEL_1
#define CHANNEL_LED            LEDC_CHANNEL_2

static const char *TAG = "LINE_FOLLOWER";

// ===== IR REMOTO =====
/*
IRrecv irrecv(PIN_IR);
decode_results results;
*/
bool robot_encendido = false;
int64_t ultimo_ir = 0;

// ===== Parámetros =====
int vel_maxima = 110;
int vel_minima = 55;
int vel_busqueda = 55;

float Kp = 0.265;
float Kd = 15;
int umbral_linea = 500;

// ===== Variables =====
int s_min[16];
int s_max[16];
int s_norm[16];

float error_val = 0;
float error_anterior = 0;
float posicion_actual = 0;
float ultima_posicion = 0;

int64_t tiempo_sin_linea = 0;

// Handle para ADC
adc_oneshot_unit_handle_t adc1_handle;

// ===== FUNCIONES AUXILIARES =====
int64_t millis_esp(void) {
    return esp_timer_get_time() / 1000;
}

int constrain_val(int val, int min_v, int max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

float constrain_f(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

// ===== Motores (INTOCABLE) =====
void ledc_write_channel(ledc_channel_t channel, int duty) {
    duty = constrain_val(duty, 0, 255);
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void hardware_motores(int izq, int der){
    if(izq >= 0){ 
        gpio_set_level(PIN_DIR_IZQ, 1); 
    } else { 
        gpio_set_level(PIN_DIR_IZQ, 0); 
        izq = -izq; 
    }
    ledc_write_channel(CHANNEL_PWM_I, izq);

    if(der >= 0){ 
        gpio_set_level(PIN_DIR_DER, 1); 
    } else { 
        gpio_set_level(PIN_DIR_DER, 0); 
        der = -der; 
    }
    ledc_write_channel(CHANNEL_PWM_D, der);
}

void moverRobot(int vel_izq, int vel_der) {
    hardware_motores(-vel_izq, vel_der); 
}

// ===== Selección de multiplexor =====
void seleccionar_canal_mux(int i) {
    gpio_set_level(PIN_S0, (i >> 0) & 1);
    gpio_set_level(PIN_S1, (i >> 1) & 1);
    gpio_set_level(PIN_S2, (i >> 2) & 1);
    gpio_set_level(PIN_S3, (i >> 3) & 1);
    esp_rom_delay_us(5);
}

int leer_adc_com() {
    int raw = 0;
    // Asumiendo ADC1 canal correspondiente al pin COM (ajustar canal según GPIO 1)
    // GPIO 1 corresponde por lo general a ADC1_CHANNEL_0 en ESP32-S3/C3/etc.
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw);
    return raw;
}

// ===== Lectura de Sensores =====
int leerSensores(){
    long suma_p = 0;
    long suma = 0;
    int detectados = 0;

    int ultimo_activo = -1;
    int gap_max = 0;

    for(int i = 0; i < 16; i++){
        seleccionar_canal_mux(i);
        int raw = leer_adc_com();

        // Mapeo equivalente a map(raw, s_min[i], s_max[i], 0, 1000)
        int val = 0;
        if (s_max[i] != s_min[i]) {
            val = (raw - s_min[i]) * 1000 / (s_max[i] - s_min[i]);
        }
        val = constrain_val(val, 0, 1000);
        s_norm[i] = val;

        if(val > umbral_linea){
            if(ultimo_activo >= 0){
                int gap = i - ultimo_activo;
                if(gap > gap_max) gap_max = gap;
            }
            ultimo_activo = i;

            long peso = (i - 7) * 1000; // Ajustado por enteros (7.5 aprox o escala directa)
            suma_p += peso * val;
            suma += val;
            detectados++;
        }
    }

    if(detectados >= 4 && gap_max >= 4){
        suma_p = 0;
        suma = 0;
        for(int i = 0; i < 16; i++){
            if(s_norm[i] > umbral_linea){
                bool usar = false;
                if(ultima_posicion <= 0 && i < 8) usar = true;
                if(ultima_posicion > 0 && i >= 8) usar = true;
                if(usar){
                    long peso = (i - 7) * 1000;
                    suma_p += peso * s_norm[i];
                    suma += s_norm[i];
                }
            }
        }
        if(suma == 0){
            posicion_actual = ultima_posicion;
            return 1;
        }
        posicion_actual = (float)suma_p / suma;
        ultima_posicion = posicion_actual;
        tiempo_sin_linea = millis_esp();
        moverRobot((int)(55 - (posicion_actual * Kp)), (int)(55 + (posicion_actual * Kp)));
        return 1;
    }

    if(detectados > 0){
        posicion_actual = (float)suma_p / suma;
        ultima_posicion = posicion_actual;
        tiempo_sin_linea = millis_esp();
    } else {
        int64_t perdido = millis_esp() - tiempo_sin_linea;
        if(perdido < 150){
            posicion_actual = ultima_posicion;
            return 1;
        }

        if(ultima_posicion > 0){
            moverRobot(vel_busqueda, -vel_busqueda);
        } else {
            moverRobot(-vel_busqueda, vel_busqueda);
        }
        return 0;
    }

    return 1;
}

// ===== Inicialización del Sistema =====
void setup_hardware(void) {
    // Configuración de Pines GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DIR_IZQ) | (1ULL << PIN_DIR_DER) | 
                        (1ULL << PIN_S0) | (1ULL << PIN_S1) | 
                        (1ULL << PIN_S2) | (1ULL << PIN_S3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(PIN_DIR_IZQ, 0);
    gpio_set_level(PIN_DIR_DER, 0);

    // Configuración ADC Oneshot
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    // Nota: Reemplazar ADC_CHANNEL_0 si el pin COM (GPIO 1) utiliza otro canal en tu modelo de ESP32
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);

    // Configuración PWM (LEDC)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY_PWM,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channels[] = {
        { .channel = CHANNEL_PWM_I, .duty = 0, .gpio_num = PIN_PWM_I, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .hpoint = 0 },
        { .channel = CHANNEL_PWM_D, .duty = 0, .gpio_num = PIN_PWM_D, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .hpoint = 0 },
        { .channel = CHANNEL_LED,   .duty = 0, .gpio_num = PIN_LED,   .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .hpoint = 0 }
    };
    
    // Ajustar frecuencia para LED si es necesario separada, o usar la misma con diferentes duty
    for (int i = 0; i < 3; i++) {
        ledc_channel_config(&ledc_channels[i]);
    }

    moverRobot(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Inicializar Infrarrojo
    //REVISAR: irrecv.enableIRIn();

    for(int i = 0; i < 16; i++){
        s_min[i] = 4095;
        s_max[i] = 0;
    }

    ESP_LOGI(TAG, "CALIBRANDO...");
    ledc_write_channel(CHANNEL_LED, 255);

    int64_t t0 = millis_esp();
    while(millis_esp() - t0 < 5000){
        for(int i = 0; i < 16; i++){
            seleccionar_canal_mux(i);
            int lectura = leer_adc_com();

            if(lectura < s_min[i]) s_min[i] = lectura;
            if(lectura > s_max[i]) s_max[i] = lectura;
        }
    }

    ledc_write_channel(CHANNEL_LED, 0);
    ESP_LOGI(TAG, "LISTO");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// ===== Loop Principal en Tarea de FreeRTOS =====
void app_main(void) {
    setup_hardware();

    while (1) {
        // ===== CONTROL REMOTO =====
        if ( 0 /*irrecv.decode(&results)*/) {
            ESP_LOGI(TAG, "Codigo: %lX", 1/*REVISAR results.value*/);

            if ( 0 /*results.value == 0xFFA25D*/) { 
                if (millis_esp() - ultimo_ir > 500) {
                    robot_encendido = !robot_encendido;
                    ultimo_ir = millis_esp();
                }
            }
            //REVISAR: irrecv.resume();
        }

        // ===== APAGADO =====
        if (!robot_encendido) {
            moverRobot(0, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ===== TU LÓGICA ORIGINAL =====
        if(!leerSensores()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        error_val = -posicion_actual;

        float P = error_val * Kp;
        float D = (error_val - error_anterior);
        D = D * 1.0f;
        D = D * Kd;

        float correccion = P + D;

        float limite = 45.0f + (abs( (int) error_val) * 0.015f); //Se hizo cast a int para evitar error, pero error_val originalmente es float
        if(limite > 90) limite = 90;
        if(correccion > limite) correccion = limite;
        if(correccion < -limite) correccion = -limite;

        error_anterior = error_val;

        int vel = vel_maxima - (int)(abs ((int)error_val) * 0.08f); //Se hizo cast a int para evitar error, pero error_val originalmente es float
        vel = constrain_val(vel, vel_minima, vel_maxima);

        int vel_izq = vel - (int)correccion;
        int vel_der = vel + (int)correccion;

        vel_izq = constrain_val(vel_izq, -20, vel_maxima);
        vel_der = constrain_val(vel_der, -20, vel_maxima);

        moverRobot(vel_izq, vel_der);
        
        // Pequeño respiro para evitar saturar el watchdog de FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}