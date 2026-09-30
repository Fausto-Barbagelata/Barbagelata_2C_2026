/**
 * @file MedirDistanciaUltraV3.c
 * @brief Actividad 3 - Medidor de distancia por ultrasonido con interrupciones y puerto serie.
 *
 * @mainpage Proyecto 2 - Actividad 3: Medidor de distancia por ultrasonido c/interrupciones y puerto serie
 *
 * @section desc Descripción
 * Extiende la Actividad 2 agregando comunicación por UART con la PC. Las mediciones se
 * envían a un terminal (por ejemplo, la extensión "Serial Monitor" de VSCode) con el formato:
 * 3 dígitos ASCII + espacio + unidad (cm / inch) + "\r\n".
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
#include "uart_mcu.h"

/*==================[macros and definitions]=================================*/
/** @brief Período inicial de medición en microsegundos (1 s). */
#define CONFIG_PERIOD_MEDICION_US_INIT 1000000 // 1 segundo (1000 ms)
/** @brief Período mínimo de medición en microsegundos (100 ms). */
#define CONFIG_PERIOD_MIN_US           100000  // Límite mínimo: 100 ms
/** @brief Paso de ajuste del período de medición en microsegundos (100 ms). */
#define CONFIG_PERIOD_STEP_US          100000  // Paso de ajuste: 100 ms
/** @brief Velocidad de la UART hacia la PC, en baudios. */
#define CONFIG_UART_BAUDRATE           115200

/*==================[internal data definition]===============================*/
/** @brief Handle de la tarea de medición con el HC-SR04. */
TaskHandle_t read_ultra_task_handle = NULL;
/** @brief Handle de la tarea que actualiza LEDs, LCD y UART. */
TaskHandle_t leds_lcd_task_handle = NULL;

// Variables Globales
/** @brief Estado de la medición: true = midiendo, false = detenida (TEC1 / tecla 'O'). */
bool medir = true;
/** @brief Estado de HOLD: true = LCD y UART congelados (TEC2 / tecla 'H'). */
bool hold = false;
/** @brief Unidad de visualización: false = cm, true = pulgadas (tecla 'I'). */
bool en_pulgadas = false;   // false = cm, true = pulgadas
/** @brief Valor a mostrar: false = valor actual, true = valor máximo (tecla 'M'). */
bool mostrar_max = false;   // false = valor actual, true = valor máximo
/** @brief Última distancia medida, en centímetros. */
uint16_t distancia = 0;
/** @brief Máxima distancia medida, en centímetros. */
uint16_t distancia_max = 0;
/** @brief Período actual del timer de medición, en microsegundos (modificable con 'F' y 'S'). */
uint32_t periodo_medicion_us = CONFIG_PERIOD_MEDICION_US_INIT;
/*==================[internal functions declaration]=========================*/
void FuncTEC1(void *param);
void FuncTEC2(void *param);
void FuncUART(void *param);
void FuncTimerMedicion(void *param);

/*==================[internal functions declaration]=========================*/
/**
 * @brief Callback de la interrupción de TEC1: activa/detiene la medición.
 */
void FuncTEC1(void *param) {
    medir = !medir;
}

/**
 * @brief Callback de la interrupción de TEC2: activa/desactiva el HOLD.
 */
void FuncTEC2(void *param) {
    hold = !hold;
}

/**
 * @brief Callback de recepción de la UART conectada a la PC.
 *
 * Lee un byte y ejecuta el comando correspondiente (no distingue mayúsculas/minúsculas):
 * - 'O': conmuta @ref medir (equivale a TEC1).
 * - 'H': conmuta @ref hold (equivale a TEC2).
 * - 'I': conmuta @ref en_pulgadas (cm / pulgadas).
 * - 'M': conmuta @ref mostrar_max; al activarlo carga @ref distancia_max con @ref distancia.
 * - 'F': reduce @ref periodo_medicion_us en 100 ms (mínimo 100 ms) y reinicia el TIMER_A.
 * - 'S': aumenta @ref periodo_medicion_us en 100 ms y reinicia el TIMER_A.
 *
 */
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

/**
 * @brief Callback del TIMER_A, ejecutado en contexto de interrupción.
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
 * Se bloquea hasta recibir la notificación del timer. Si @ref medir es true, actualiza
 * @ref distancia (cm) y, si corresponde, el máximo @ref distancia_max.
 *
 */
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

/**
 * @brief Tarea que muestra la distancia en LEDs, display LCD y puerto serie.
 *
 * Se bloquea hasta recibir la notificación del timer.
 * - Con @ref medir activo: maneja los LEDs según @ref distancia (siempre en cm). Si @ref hold
 *   está inactivo, elige el valor (actual o máximo), lo convierte a pulgadas si
 *   @ref en_pulgadas es true (1 pulgada = 2.54 cm), lo escribe en el LCD y lo envía por la UART
 *   con formato "%03u cm\r\n" o "%03u inch\r\n".
 * - Con @ref medir inactivo: apaga los LEDs y, si @ref hold está inactivo, apaga el LCD.
 *
 */
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

/**
 * @brief Función principal de la aplicación.
 *
 * Inicializa LEDs, teclas, HC-SR04 (ECHO = GPIO_3, TRIGGER = GPIO_2) y LCD; habilita las
 * interrupciones de TEC1 y TEC2; inicializa la UART_PC a @ref CONFIG_UART_BAUDRATE con
 * @ref FuncUART como callback de recepción; crea las tareas de FreeRTOS y arranca el TIMER_A
 * con el período inicial @ref periodo_medicion_us.
 */
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