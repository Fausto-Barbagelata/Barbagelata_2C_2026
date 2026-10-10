/**
 * @file ProyectoIntegrador.c
 * @brief Adquisicion de acelerometria ADXL345 con calibracion dinamica de offset,
 *        seleccion de eje activo y filtrado IIR [1 Hz - 600 Hz].
 *
 * Diseno modular orientado a FreeRTOS:
 * - Alta cohesion y bajo acoplamiento mediante Queues inter-tareas.
 * - Calibracion inicial de offset DC (gravedad) durante 10 segundos (16000 muestras).
 * - Seleccion configurable de eje a transmitir (Eje Y por defecto).
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "adxl345.h"
#include "iir_filter.h"

/*==================[macros and definitions]=================================*/
#define UART_BAUD_RATE          921600
#define SAMPLE_FREQ_HZ          1600
#define SAMPLE_PERIOD_US        (1000000 / SAMPLE_FREQ_HZ)

/** @brief Duracion de la calibracion inicial en segundos */
#define CALIBRATION_TIME_SEC    10
/** @brief Cantidad total de muestras para la rutina de calibracion de 10s (16000 muestras) */
#define CALIBRATION_SAMPLES     (SAMPLE_FREQ_HZ * CALIBRATION_TIME_SEC)

/** @brief Enumeracion para la seleccion de eje a monitorear */
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_ALL
} active_axis_t;

/** @brief Eje activo seleccionado para la visualizacion (Cambiar segun necesidad) */
#define SELECTED_AXIS           AXIS_Z

/** @brief Estructura de datos para transferir muestras entre tareas */
typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

/*==================[internal data definition]===============================*/
static TaskHandle_t sample_task_handle = NULL;

/* Queues para desacoplar el pipeline de procesamiento */
static QueueHandle_t raw_data_queue = NULL;
static QueueHandle_t filtered_data_queue = NULL;

/* Estado global de calibracion de offset DC */
static float offset_x = 0.0f;
static float offset_y = 0.0f;
static float offset_z = 0.0f;
static volatile bool is_calibrated = false;

/*==================[internal functions definition]==========================*/

/**
 * @brief Callback del timer de muestreo a 1600 Hz.
 */
static void FuncTimerSample(void *param){
    vTaskNotifyGiveFromISR(sample_task_handle, pdFALSE);
}

/**
 * @brief Tarea 1: Adquisicion desde el sensor ADXL345.
 * Cohesion: Alta (Solo lee del sensor y encola los datos).
 */
static void SampleTask(void *param){
    adxl345_accel_raw_t raw;
    accel_data_t sample;

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(adxl345_read_accel(&raw) == ADXL345_OK){
            sample.x = (float)adxl345_raw_to_mg(raw.x, true, ADXL345_RANGE_2G);
            sample.y = (float)adxl345_raw_to_mg(raw.y, true, ADXL345_RANGE_2G);
            sample.z = (float)adxl345_raw_to_mg(raw.z, true, ADXL345_RANGE_2G);

            xQueueSend(raw_data_queue, &sample, 0);
        }
    }
}

/**
 * @brief Tarea 2: Calibracion inicial de offset (10 segundos).
 * Cohesion: Alta (Calcula el valor medio inicial para centrar la senal en 0 mg).
 */
static void CalibrationTask(void *param){
    accel_data_t sample;
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint32_t count = 0;

    printf("Iniciando calibracion de offset (10 segundos). Mantenga el sensor en reposo...\r\n");

    while(count < CALIBRATION_SAMPLES){
        if(xQueueReceive(raw_data_queue, &sample, portMAX_DELAY) == pdTRUE){
            sum_x += sample.x;
            sum_y += sample.y;
            sum_z += sample.z;
            count++;
        }
    }

    offset_x = (float)(sum_x / CALIBRATION_SAMPLES);
    offset_y = (float)(sum_y / CALIBRATION_SAMPLES);
    offset_z = (float)(sum_z / CALIBRATION_SAMPLES);

    is_calibrated = true;
    printf("Calibracion finalizada. Offsets -> X: %.2f mg, Y: %.2f mg, Z: %.2f mg\r\n",
           offset_x, offset_y, offset_z);

    vTaskDelete(NULL);
}

/**
 * @brief Tarea 3: Procesamiento, resta de offset y filtrado IIR.
 * Cohesion: Alta (Aplica algoritmos de procesamiento digital de senales).
 */
static void ProcessingTask(void *param){
    accel_data_t sample;
    accel_data_t filtered_sample;

    /* Espera a que finalice la tarea de calibracion */
    while(!is_calibrated){
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while(true){
        if(xQueueReceive(raw_data_queue, &sample, portMAX_DELAY) == pdTRUE){

            /* 1. Resta del valor medio (Offset DC / Gravedad) */
            sample.x -= offset_x;
            sample.y -= offset_y;
            sample.z -= offset_z;

            /* 2. Seleccion y filtrado segun el eje activo configurado */
            switch(SELECTED_AXIS){
                case AXIS_X:
                    HiPassFilter(&sample.x, &filtered_sample.x, 1);
                    LowPassFilter(&filtered_sample.x, &filtered_sample.x, 1);
                    break;

                case AXIS_Y:
                    HiPassFilter(&sample.y, &filtered_sample.y, 1);
                    LowPassFilter(&filtered_sample.y, &filtered_sample.y, 1);
                    break;

                case AXIS_Z:
                    HiPassFilter(&sample.z, &filtered_sample.z, 1);
                    LowPassFilter(&filtered_sample.z, &filtered_sample.z, 1);
                    break;

                case AXIS_ALL:
                    HiPassFilter(&sample.x, &filtered_sample.x, 1);
                    LowPassFilter(&filtered_sample.x, &filtered_sample.x, 1);
                    HiPassFilter(&sample.y, &filtered_sample.y, 1);
                    LowPassFilter(&filtered_sample.y, &filtered_sample.y, 1);
                    HiPassFilter(&sample.z, &filtered_sample.z, 1);
                    LowPassFilter(&filtered_sample.z, &filtered_sample.z, 1);
                    break;
            }

            xQueueSend(filtered_data_queue, &filtered_sample, 0);
        }
    }
}

/**
 * @brief Tarea 4: Transmision UART hacia el Serial Plotter.
 * Cohesion: Alta (Formateo y salida por puerto serie).
 */
static void UartTxTask(void *param){
    accel_data_t sample;
    char linea[64];

    while(true){
        if(xQueueReceive(filtered_data_queue, &sample, portMAX_DELAY) == pdTRUE){
            switch(SELECTED_AXIS){
                case AXIS_X:
                    snprintf(linea, sizeof(linea), "%.2f\r\n", sample.x);
                    break;

                case AXIS_Y:
                    snprintf(linea, sizeof(linea), "%.2f\r\n", sample.y);
                    break;

                case AXIS_Z:
                    snprintf(linea, sizeof(linea), "%.2f\r\n", sample.z);
                    break;

                case AXIS_ALL:
                    snprintf(linea, sizeof(linea), "%.2f,%.2f,%.2f\r\n", sample.x, sample.y, sample.z);
                    break;
            }

            UartSendString(UART_PC, linea);
        }
    }
}

/*==================[external functions definition]==========================*/

void app_main(void){

    /* --- Inicializacion del sensor ADXL345 --- */
    adxl345_config_t adxl_config = {
        .range                  = ADXL345_RANGE_2G,
        .full_resolution        = true,
        .rate_code              = ADXL345_RATE_1600HZ,
        .low_power_mode         = false,
        .use_fifo_stream        = false,
        .fifo_watermark_samples = 0,
    };

    if(adxl345_init(&adxl_config) != ADXL345_OK){
        printf("ERROR: no se pudo inicializar el ADXL345 (revisar SPI/DEVID)\r\n");
        return;
    }

    /* --- Inicializacion de Filtros IIR (1 Hz - 600 Hz) --- */
    HiPassInit(SAMPLE_FREQ_HZ, 1, ORDER_2);
    LowPassInit(SAMPLE_FREQ_HZ, 600, ORDER_2);

    /* --- Inicializacion UART --- */
    serial_config_t uart_config = {
        .port      = UART_PC,
        .baud_rate = UART_BAUD_RATE,
        .func_p    = UART_NO_INT,
        .param_p   = NULL
    };
    UartInit(&uart_config);

    /* --- Creacion de Queues de comunicacion inter-tareas --- */
    raw_data_queue = xQueueCreate(16, sizeof(accel_data_t));
    filtered_data_queue = xQueueCreate(16, sizeof(accel_data_t));

    /* --- Creacion de Tareas de FreeRTOS --- */
    xTaskCreate(&SampleTask, "sample_adxl345", 2048, NULL, 6, &sample_task_handle);
    xTaskCreate(&CalibrationTask, "calibration", 2048, NULL, 5, NULL);
    xTaskCreate(&ProcessingTask, "processing", 4096, NULL, 4, NULL);
    xTaskCreate(&UartTxTask, "uart_tx", 2048, NULL, 3, NULL);

    /* --- Configuración y arranque de Timer --- */
    timer_config_t timer_sample = {
        .timer   = TIMER_A,
        .period  = SAMPLE_PERIOD_US,
        .func_p  = FuncTimerSample,
        .param_p = NULL
    };
    TimerInit(&timer_sample);
    TimerStart(timer_sample.timer);
}

/*==================[end of file]============================================*/