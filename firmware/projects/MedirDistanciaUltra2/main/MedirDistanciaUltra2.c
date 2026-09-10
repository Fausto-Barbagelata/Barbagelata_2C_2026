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
#define CONFIG_PERIOD_MEDICION_US 1000000

/*==================[internal data definition]===============================*/
TaskHandle_t read_ultra_task_handle = NULL;
TaskHandle_t leds_lcd_task_handle = NULL;

// Variables Globales
bool MEDIR = true;
bool HOLD = false;
uint16_t DISTANCIA = 0;

/*==================[internal functions declaration]========= ===============*/
void FuncTEC1(void *param) {
    MEDIR = !MEDIR;
}

void FuncTEC2(void *param) {
    HOLD = !HOLD;
}

void FuncTimerMedicion(void *param) {
    vTaskNotifyGiveFromISR(read_ultra_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(leds_lcd_task_handle, pdFALSE);
}

static void ReadUltraTask(void *pvParameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (MEDIR) {
            DISTANCIA = HcSr04ReadDistanceInCentimeters();
        }
    }
}

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