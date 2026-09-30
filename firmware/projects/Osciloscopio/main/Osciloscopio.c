/**
 * @file Osciloscopio.c
 * @brief Actividad 4 - Osciloscopio con ESP32-C6 (prueba con señal de ECG)
 *
 * @mainpage Proyecto 2 - Actividad 4: Osciloscopio
 *
 * @section desc Descripción
 * Aplicación que digitaliza una señal analógica y la transmite a un graficador por puerto
 * serie de la PC (extensión "Serial Plotter" de VSCode). Se basa en los drivers
 * analog_io_mcu.h y uart_mcu.h.
 *
 * - TIMER_B (250 Hz): reproduce el ECG (provisto por la cátedra) por la salida analógica
 *   (DAC/SDM, GPIO0).
 * - TIMER_A (500 Hz): dispara la conversión A/D del canal CH1 (GPIO1).
 * - La muestra se formatea con snprintf() y se envía por UART_PC en formato compatible con
 *   Serial Plotter: `>CH1:<valor>\r\n` (se ignoran datos sin el carácter ">").
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
#include "analog_io_mcu.h"
#include "uart_mcu.h"
#include "timer_mcu.h"

/*==================[macros and definitions]=================================*/
/** @brief Velocidad de la UART hacia la PC, en baudios. */
#define UART_BAUD_RATE      115200
/** @brief Canal del conversor A/D utilizado para digitalizar la señal. */
#define ADC_CHANNEL         CH1

/** @brief Frecuencia de muestreo del ADC en Hz. */
#define SAMPLE_FREQ_HZ      500                         /* muestreo del ADC  */
/** @brief Período de muestreo del ADC en microsegundos (2000 us). */
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_FREQ_HZ)  /* 2000 us           */

/** @brief Frecuencia de reproducción de las muestras del ECG en Hz. */
#define ECG_FREQ_HZ         250                         /* reproducción ECG  */
/** @brief Período de reproducción del ECG en microsegundos (4000 us). */
#define ECG_PERIOD_US       (1000000 / ECG_FREQ_HZ)     /* 4000 us           */
/** @brief Cantidad de muestras del vector de ECG. */
#define BUFFER_SIZE         231

/*==================[internal data definition]===============================*/
/** @brief Handle de la tarea que muestrea el ADC y envía por UART. */
static TaskHandle_t sample_task_handle = NULL;
/** @brief Handle de la tarea que reproduce el ECG por la salida analógica. */
static TaskHandle_t ecg_task_handle    = NULL;

/** @brief Señal digital de ECG (provista por la cátedra), 231 muestras de 8 bits. */
const char ecg[BUFFER_SIZE] = {
    76, 77, 78, 77, 79, 86, 81, 76, 84, 93, 85, 80,
    89, 95, 89, 85, 93, 98, 94, 88, 98, 105, 96, 91,
    99, 105, 101, 96, 102, 106, 101, 96, 100, 107, 101,
    94, 100, 104, 100, 91, 99, 103, 98, 91, 96, 105, 95,
    88, 95, 100, 94, 85, 93, 99, 92, 84, 91, 96, 87, 80,
    83, 92, 86, 78, 84, 89, 79, 73, 81, 83, 78, 70, 80, 82,
    79, 69, 80, 82, 81, 70, 75, 81, 77, 74, 79, 83, 82, 72,
    80, 87, 79, 76, 85, 95, 87, 81, 88, 93, 88, 84, 87, 94,
    86, 82, 85, 94, 85, 82, 85, 95, 86, 83, 92, 99, 91, 88,
    94, 98, 95, 90, 97, 105, 104, 94, 98, 114, 117, 124, 144,
    180, 210, 236, 253, 227, 171, 99, 49, 34, 29, 43, 69, 89,
    89, 90, 98, 107, 104, 98, 104, 110, 102, 98, 103, 111, 101,
    94, 103, 108, 102, 95, 97, 106, 100, 92, 101, 103, 100, 94, 98,
    103, 96, 90, 98, 103, 97, 90, 99, 104, 95, 90, 99, 104, 100, 93,
    100, 106, 101, 93, 101, 105, 103, 96, 105, 112, 105, 99, 103, 108,
    99, 96, 102, 106, 99, 90, 92, 100, 87, 80, 82, 88, 77, 69, 75, 79,
    74, 67, 71, 78, 72, 67, 73, 81, 77, 71, 75, 84, 79, 77, 77, 76, 76,
};

/*==================[internal functions definition]==========================*/

/**
 * @brief Callback del TIMER_A (500 Hz), ejecutado en contexto de interrupción.
 *
 * Notifica a @ref SampleTask para que tome una muestra del ADC.
 *
 */
static void FuncTimerSample(void *param){
    vTaskNotifyGiveFromISR(sample_task_handle, pdFALSE);
}

/**
 * @brief Callback del TIMER_B (250 Hz), ejecutado en contexto de interrupción.
 *
 * Notifica a @ref EcgTask para que envíe la siguiente muestra del ECG al DAC.
 *
 */
static void FuncTimerEcg(void *param){
    vTaskNotifyGiveFromISR(ecg_task_handle, pdFALSE);
}

/** Lee CH1 y envía ">CH1:<valor>\r\n" (armado con snprintf). */
/**
 * @brief Tarea de muestreo y transmisión.
 *
 * Al recibir la notificación del TIMER_A lee el canal CH1 del ADC, arma la línea
 * `>CH1:<valor>\r\n` con snprintf() y la envía por UART_PC para ser graficada en
 * Serial Plotter.
 *
 */
static void SampleTask(void *param){
    uint16_t sample = 0;
    char linea[32];

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AnalogInputReadSingle(ADC_CHANNEL, &sample);

        snprintf(linea, sizeof(linea), "%d\r\n", (unsigned int)sample);
        UartSendString(UART_PC, linea);
    }
}

/** Saca una muestra del ECG por el DAC en cada tick de 250 Hz. */
/**
 * @brief Tarea de reproducción del ECG.
 *
 * Al recibir la notificación del TIMER_B escribe la muestra ecg[i] en la salida analógica
 * (con cast a uint8_t porque hay valores mayores a 127) y avanza el índice de forma
 * circular sobre las @ref BUFFER_SIZE muestras.
 *
 */
static void EcgTask(void *param){
    uint16_t i = 0;

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AnalogOutputWrite((uint8_t)ecg[i]);   /* cast: hay valores > 127 */

        i++;
        if(i >= BUFFER_SIZE){
            i = 0;
        }
    }
}

/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 *
 * Configura el ADC (CH1, modo single), la salida analógica y la UART_PC (115200 baud, sin
 * interrupción de recepción), crea las tareas @ref SampleTask y @ref EcgTask (prioridad 5,
 * stack de 4096 bytes) y configura y arranca TIMER_A (@ref SAMPLE_PERIOD_US) y TIMER_B
 * (@ref ECG_PERIOD_US).
 */
void app_main(void){

    analog_input_config_t adc_config = {
        .input       = ADC_CHANNEL,
        .mode        = ADC_SINGLE,
        .func_p      = NULL,
        .param_p     = NULL,
        .sample_frec = 0
    };
    AnalogInputInit(&adc_config);

    AnalogOutputInit();

    serial_config_t uart_config = {
        .port      = UART_PC,
        .baud_rate = UART_BAUD_RATE,
        .func_p    = UART_NO_INT,
        .param_p   = NULL
    };
    UartInit(&uart_config);

    xTaskCreate(&SampleTask, "sample", 2048, NULL, 5, &sample_task_handle);
    xTaskCreate(&EcgTask,    "ecg",    2048, NULL, 5, &ecg_task_handle);

    timer_config_t timer_sample = {
        .timer   = TIMER_A,
        .period  = SAMPLE_PERIOD_US,
        .func_p  = FuncTimerSample,
        .param_p = NULL
    };
    TimerInit(&timer_sample);

    timer_config_t timer_ecg = {
        .timer   = TIMER_B,
        .period  = ECG_PERIOD_US,
        .func_p  = FuncTimerEcg,
        .param_p = NULL
    };
    TimerInit(&timer_ecg);

    TimerStart(timer_ecg.timer);
    TimerStart(timer_sample.timer);
}

/*==================[end of file]============================================*/