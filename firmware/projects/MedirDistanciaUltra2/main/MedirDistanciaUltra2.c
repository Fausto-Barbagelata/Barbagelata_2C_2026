/**
 * @file MedirDistanciaUltra2.c
 * @brief Actividad 2 - Medidor de distancia por ultrasonido con interrupciones.
 *
 * @mainpage Proyecto 2 - Actividad 2: Medidor de distancia por ultrasonido c/interrupciones
 *
 * @section desc Descripción
 * Modificación de la Actividad 1 para utilizar interrupciones en el control de las teclas
 * y en el control de tiempos (Timer).
 * 
 * @author Fausto Benjamin Barbagelata
 *
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
#include "timer_mcu.h"

/*==================[macros and definitions]=================================*/
/** @brief Período del timer de medición en microsegundos (1 s). */
#define CONFIG_PERIOD_MEDICION_US 1000000

/*==================[internal data definition]===============================*/
/** @brief Handle de la tarea de medición con el HC-SR04. */
TaskHandle_t read_ultra_task_handle = NULL;
/** @brief Handle de la tarea que actualiza LEDs y display LCD. */
TaskHandle_t leds_lcd_task_handle = NULL;

// Variables Globales
/** @brief Estado de la medición: true = midiendo, false = detenida (TEC1). */
bool MEDIR = true;
/** @brief Estado de HOLD: true = valor del LCD congelado (TEC2). */
bool HOLD = false;
/** @brief Última distancia medida, en centímetros. */
uint16_t DISTANCIA = 0;

/*==================[internal functions declaration]========= ===============*/
/**
 * @brief Callback de la interrupción de TEC1: activa/detiene la medición.
 */
void FuncTEC1(void *param) {
    MEDIR = !MEDIR;
}

/**
 * @brief Callback de la interrupción de TEC2: activa/desactiva el HOLD.
 */
void FuncTEC2(void *param) {
    HOLD = !HOLD;
}

/**
 * @brief Callback del TIMER_A (cada 1 s), ejecutado en contexto de interrupción.
 *
 * Notifica a @ref ReadUltraTask y a @ref LedsAndLcdTask para que realicen un ciclo.
 *
 */
void FuncTimerMedicion(void *param) {
    vTaskNotifyGiveFromISR(read_ultra_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(leds_lcd_task_handle, pdFALSE);
}

/**
 * @brief Tarea de medición de distancia.
 *
 * Se bloquea hasta recibir la notificación del timer. Si @ref MEDIR es true, actualiza
 * @ref DISTANCIA con la medición del HC-SR04 en cm.
 *
 */
static void ReadUltraTask(void *pvParameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (MEDIR) {
            DISTANCIA = HcSr04ReadDistanceInCentimeters();
        }
    }
}

/**
 * @brief Tarea que muestra la distancia en LEDs y display LCD.
 *
 * Se bloquea hasta recibir la notificación del timer.
 * - Con @ref MEDIR activo: enciende los LEDs según @ref DISTANCIA y, si @ref HOLD está
 *   inactivo, escribe el valor en el LCD.
 * - Con @ref MEDIR inactivo: apaga los LEDs y, si @ref HOLD está inactivo, apaga el LCD.
 *
 */
static void LedsAndLcdTask(void *pvParameter) {
    LcdItsE0803Off();
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
    }
}

/*==================[external functions definition]==========================*/

/**
 * @brief Función principal de la aplicación.
 *
 * Inicializa periféricos (LEDs, teclas, HC-SR04 con ECHO = GPIO_3 y TRIGGER = GPIO_2, LCD),
 * habilita las interrupciones de TEC1 y TEC2, crea las tareas de FreeRTOS y configura y
 * arranca el TIMER_A con período de 1 s.
 */
void app_main(void) {
    LedsInit();
    SwitchesInit();
    HcSr04Init(GPIO_3, GPIO_2); // ECHO = GPIO_3, TRIGGER = GPIO_2
    LcdItsE0803Init();

    // Configuración de Interrupciones para las Teclas TEC1 y TEC2
    SwitchActivInt(SWITCH_1, &FuncTEC1, NULL);
    SwitchActivInt(SWITCH_2, &FuncTEC2, NULL);

    // Creación de Tareas FreeRTOS
    xTaskCreate(&ReadUltraTask, "ReadUltraTask", 2048, NULL, 5, &read_ultra_task_handle);
    xTaskCreate(&LedsAndLcdTask, "LedsAndLcdTask", 2048, NULL, 5, &leds_lcd_task_handle);

    // Configuración e Inicialización del Timer A de Hardware (1 segundo)
    // Para otras aplicaciones podriamos usar mas de un Timer, en este caso no tiene sentido
    // hacer 2 distintos con el mismo periodo asi que solo usamos 1.
    timer_config_t timer_medicion = {
        .timer = TIMER_A,
        .period = CONFIG_PERIOD_MEDICION_US,
        .func_p = FuncTimerMedicion,
        .param_p = NULL
    };
    TimerInit(&timer_medicion);
    TimerStart(timer_medicion.timer);
}