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
#include "uart_mcu.h"

/*==================[macros and definitions]=================================*/
#define CONFIG_PERIOD_MEDICION_US_INIT 1000000 // 1 segundo (1000 ms)
#define CONFIG_PERIOD_MIN_US           100000  // Límite mínimo: 100 ms
#define CONFIG_PERIOD_STEP_US          100000  // Paso de ajuste: 100 ms
#define CONFIG_UART_BAUDRATE           115200

/*==================[internal data definition]===============================*/
TaskHandle_t read_ultra_task_handle = NULL;
TaskHandle_t leds_lcd_task_handle = NULL;

// Variables Globales
bool medir = true;
bool hold = false;
bool en_pulgadas = false;   // false = cm, true = pulgadas
bool mostrar_max = false;   // false = valor actual, true = valor máximo
uint16_t distancia = 0;
uint16_t distancia_max = 0;
uint32_t periodo_medicion_us = CONFIG_PERIOD_MEDICION_US_INIT;
/*==================[internal functions declaration]=========================*/
void FuncTEC1(void *param);
void FuncTEC2(void *param);
void FuncUART(void *param);
void FuncTimerMedicion(void *param);

/*==================[internal functions declaration]=========================*/
void FuncTEC1(void *param) {
    medir = !medir;
}

void FuncTEC2(void *param) {
    hold = !hold;
}

// Callback de recepción UART
void FuncUART(void *param) {
    uint8_t dato;
    if (UartReadByte(UART_PC, &dato)) {
        switch (dato) {
            case 'O':
            case 'o':
                medir = !medir;
                break;
            case 'H':
            case 'h':
                hold = !hold;
                break;
            case 'I':
            case 'i':
                en_pulgadas = !en_pulgadas;
                break;
            case 'M':
            case 'm':
                mostrar_max = !mostrar_max;
                if (mostrar_max) {
                 distancia_max = distancia;
                 }
                break;
            case 'F':
            case 'f':
                // Aumentar velocidad -> reducir período (mínimo 100 ms)
                if (periodo_medicion_us > CONFIG_PERIOD_MIN_US) {
                    periodo_medicion_us -= CONFIG_PERIOD_STEP_US;
                    timer_config_t timer_medicion = {
                        .timer = TIMER_A,
                        .period = periodo_medicion_us,
                        .func_p = FuncTimerMedicion,
                        .param_p = NULL
                    };
                    TimerInit(&timer_medicion);
                    TimerStart(timer_medicion.timer);
                }
                break;
            case 'S':
            case 's':
                // Disminuir velocidad -> aumentar período
                periodo_medicion_us += CONFIG_PERIOD_STEP_US;
                timer_config_t timer_medicion_s = {
                    .timer = TIMER_A,
                    .period = periodo_medicion_us,
                    .func_p = FuncTimerMedicion,
                    .param_p = NULL
                };
                TimerInit(&timer_medicion_s);
                TimerStart(timer_medicion_s.timer);
                break;
            default:
                break;
        }
    }
}

void FuncTimerMedicion(void *param) {
    vTaskNotifyGiveFromISR(read_ultra_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(leds_lcd_task_handle, pdFALSE);
}

static void ReadUltraTask(void *pvParameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (medir) {
            distancia = HcSr04ReadDistanceInCentimeters();
            if (distancia > distancia_max) {
                distancia_max = distancia;
            }
        }
    }
}

static void LedsAndLcdTask(void *pvParameter) {
    LcdItsE0803Off();
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (medir) {
            // Manejo de LEDs siempre según la distancia real actual en cm
            if (distancia < 10) {
                LedsOffAll();
            } else if (distancia >= 10 && distancia < 20) {
                LedOn(LED_1);
                LedOff(LED_2);
                LedOff(LED_3);
            } else if (distancia >= 20 && distancia <= 30) {
                LedOn(LED_1);
                LedOn(LED_2);
                LedOff(LED_3);
            } else if (distancia > 30) {
                LedOn(LED_1);
                LedOn(LED_2);
                LedOn(LED_3);
            }

            // Si HOLD no está activo, actualizamos tanto el LCD como la salida por UART
            if (!hold) {
                // Selección del valor a mostrar (Actual o Máximo)
                uint16_t valor_mostrar = mostrar_max ? distancia_max : distancia;

                // Conversión de unidades si aplica (1 pulgada ≈ 2.54 cm)
                if (en_pulgadas) {
                    valor_mostrar = (uint16_t)((uint32_t)valor_mostrar * 100 / 254);
                }

                // 1. Escribir en el Display LCD
                LcdItsE0803Write(valor_mostrar);

                // 2. Enviar por el Serial Monitor
                char msg[32];
                if (en_pulgadas) {
                    snprintf(msg, sizeof(msg), "%03u inch\r\n", valor_mostrar);
                } else {
                    snprintf(msg, sizeof(msg), "%03u cm\r\n", valor_mostrar);
                }
                UartSendString(UART_PC, msg);
            }
        } else {
            LedsOffAll();
            if (!hold) {
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

    // Interrupciones por teclado físico
    SwitchActivInt(SWITCH_1, &FuncTEC1, NULL);
    SwitchActivInt(SWITCH_2, &FuncTEC2, NULL);

    // Inicialización UART
    serial_config_t my_uart = {
        .port = UART_PC,
        .baud_rate = CONFIG_UART_BAUDRATE,
        .func_p = &FuncUART,
        .param_p = NULL
    };
    UartInit(&my_uart);

    // Creación de Tareas FreeRTOS
    xTaskCreate(&ReadUltraTask, "ReadUltraTask", 2048, NULL, 5, &read_ultra_task_handle);
    xTaskCreate(&LedsAndLcdTask, "LedsAndLcdTask", 2048, NULL, 5, &leds_lcd_task_handle);

    // Inicialización del Timer
    timer_config_t timer_medicion = {
        .timer = TIMER_A,
        .period = periodo_medicion_us,
        .func_p = FuncTimerMedicion,
        .param_p = NULL
    };
    TimerInit(&timer_medicion);
    TimerStart(timer_medicion.timer);
}