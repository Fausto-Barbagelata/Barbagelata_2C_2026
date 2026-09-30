/**
 * @file MedirDistanciaUltra.c
 * @brief Actividad 1 - Medidor de distancia por ultrasonido
 *
 * @mainpage Proyecto 2 - Actividad 1: Medidor de distancia por ultrasonido
 *
 * @section desc Descripción
 * Firmware para la EDU-ESP (ESP32-C6) que mide distancia con un sensor HC-SR04 y la
 * muestra mediante LEDs y un display LCD (LCDITSE0803). Se utilizan los drivers provistos
 * por la cátedra (led, switch, hc_sr04, lcditse0803) sobre FreeRTOS.
 * 
 * @author Fausto Barbagelata
 * Electrónica Programable - Bioingeniería - Facultad de Ingeniería UNER.
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "hc_sr04.h"
#include "lcditse0803.h"
#include "switch.h"

/*==================[macros and definitions]=================================*/
/** @brief Período de medición del sensor de ultrasonido en milisegundos (1 s). */
#define CONFIG_BLINK_PERIOD_MEDICION 1000

/*==================[internal data definition]===============================*/
/** @brief Handle de la tarea de lectura de teclas. */
TaskHandle_t switches_task_handle = NULL;
/** @brief Handle de la tarea de medición con el sensor HC-SR04. */
TaskHandle_t read_ultra_task_handle = NULL;
/** @brief Handle de la tarea que actualiza LEDs y display LCD. */
TaskHandle_t leds_lcd_task_handle = NULL;
/** @brief Estado de la medición: true = midiendo, false = detenida (controlada por TEC1). */
bool MEDIR = true;
/** @brief Estado de HOLD: true = el valor del LCD se mantiene congelado (controlado por TEC2). */
bool HOLD = false;
/** @brief Última distancia medida, en centímetros. */
uint16_t DISTANCIA = 0;

/*==================[internal functions declaration]========= ===============*/
/**
 * @brief Tarea que lee las teclas por sondeo.
 *
 * Cada 200 ms consulta el estado de las teclas. TEC1 (SWITCH_1) conmuta @ref MEDIR y
 * TEC2 (SWITCH_2) conmuta @ref HOLD.
 *
 */
static void ReadSwitchesTask(void *pvParameter) {
    while (true) {
        int8_t tecla = SwitchesRead();
        if (tecla == SWITCH_1) {
            MEDIR = !MEDIR;
        } 
        else if (tecla == SWITCH_2) {
            HOLD = !HOLD;
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarea que mide la distancia con el HC-SR04.
 *
 * Si @ref MEDIR es true, guarda en @ref DISTANCIA el valor medido en cm. Se ejecuta
 * cada @ref CONFIG_BLINK_PERIOD_MEDICION ms (1 s).
 *
 */
static void ReadUltraTask(void *pvParameter) {
    while (true) {
        if (MEDIR) {
            DISTANCIA = HcSr04ReadDistanceInCentimeters();
        }
        
        vTaskDelay(CONFIG_BLINK_PERIOD_MEDICION / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarea que muestra la distancia en LEDs y en el display LCD.
 *
 * - Con @ref MEDIR activo: enciende los LEDs según el rango de @ref DISTANCIA
 *   (<10 cm ninguno; 10-20 LED_1; 20-30 LED_1 y LED_2; >30 los tres) y, si @ref HOLD
 *   está inactivo, escribe el valor en el LCD.
 * - Con @ref MEDIR inactivo: apaga todos los LEDs y, si @ref HOLD está inactivo, apaga el LCD.
 *
 * Se ejecuta cada 100 ms.
 *
 */
static void LedsAndLcdTask(void *pvParameter) {
    LcdItsE0803Off();
    while (true) {
        if (MEDIR) {
            if (DISTANCIA < 10) {
                LedsOffAll();
            } else if (DISTANCIA >= 10 && DISTANCIA < 20) {
                LedOn(LED_1);
                LedOff(LED_2);
                LedOff(LED_3);
            } else if (DISTANCIA >= 20 && DISTANCIA <= 30) {
                LedOn(LED_1);
                LedOn(LED_2);
                LedOff(LED_3);
            } else if (DISTANCIA > 30) {
                LedOn(LED_1);
                LedOn(LED_2);
                LedOn(LED_3);
            }
            if (!HOLD) {
                LcdItsE0803Write(DISTANCIA);
            }
        } else {
            LedsOffAll();
            if (!HOLD) {
                LcdItsE0803Off();
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*==================[external functions definition]==========================*/

/**
 * @brief Función principal de la aplicación.
 *
 * Inicializa LEDs, teclas, sensor HC-SR04 (ECHO = GPIO_3, TRIGGER = GPIO_2) y display LCD,
 * y crea las tres tareas de FreeRTOS (prioridad 5, stack de 2048 bytes cada una).
 */
void app_main(void) {
    LedsInit();
    SwitchesInit();
    HcSr04Init(GPIO_3, GPIO_2); // ECHO = GPIO_3, TRIGGER = GPIO_2
    LcdItsE0803Init();
    // Creación de Tareas FreeRTOS
    xTaskCreate(&ReadSwitchesTask, "ReadSwitchesTask", 2048, NULL, 5, &switches_task_handle);
    xTaskCreate(&ReadUltraTask, "ReadUltraTask", 2048, NULL, 5, &read_ultra_task_handle);
    xTaskCreate(&LedsAndLcdTask, "LedsAndLcdTask", 2048, NULL, 5, &leds_lcd_task_handle);
}