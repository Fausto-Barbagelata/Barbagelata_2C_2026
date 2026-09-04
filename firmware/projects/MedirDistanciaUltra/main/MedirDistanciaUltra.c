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
#define CONFIG_BLINK_PERIOD_MEDICION 1000

/*==================[internal data definition]===============================*/
TaskHandle_t switches_task_handle = NULL;
TaskHandle_t read_ultra_task_handle = NULL;
TaskHandle_t leds_lcd_task_handle = NULL;
bool MEDIR = true;
bool HOLD = false;
uint16_t DISTANCIA = 0;

/*==================[internal functions declaration]========= ===============*/
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

static void ReadUltraTask(void *pvParameter) {
    while (true) {
        if (MEDIR) {
            DISTANCIA = HcSr04ReadDistanceInCentimeters();
        }
        
        vTaskDelay(CONFIG_BLINK_PERIOD_MEDICION / portTICK_PERIOD_MS);
    }
}

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