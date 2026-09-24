/**
 * @file test_adxl345.c
 * @brief Aplicacion de prueba simple para el driver ADXL345.
 *
 * @details
 * Estructura minima: toda la logica vive en app_main(), con un unico
 * bucle infinito y un delay entre lecturas. No se crean tareas de
 * FreeRTOS propias ni se usa un driver de UART aparte: la salida se
 * hace con printf() (que en ESP-IDF sale por la consola/UART0 por
 * defecto, sin necesidad de inicializar nada extra).
 *
 * vTaskDelay() se usa unicamente como mecanismo de espera (es la forma
 * estandar de "delay" en ESP-IDF, ya que ademas alimenta al watchdog
 * del sistema); no se crean tareas adicionales ni se usan colas,
 * semaforos, etc.
 *
 * Que verifica esta prueba:
 *   1. Inicializacion del sensor y autoverificacion de DEVID (0xE5).
 *   2. Lectura periodica de aceleracion cruda (X, Y, Z) y su conversion
 *      a mili-g.
 *   3. Lectura del contador de entradas del FIFO.
 *   4. Ciclo de standby / wakeup para validar el modo de bajo consumo.
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adxl345.h"

/*==================[macros]===================================================*/
#define TEST_PERIOD_MS          200U   /*!< Periodo entre lecturas (ms) */
#define TEST_STANDBY_EVERY_N    25U    /*!< Cada cuantas lecturas se prueba standby/wakeup */

/*==================[external functions definition]============================*/

void app_main(void)
{
    printf("=== Test ADXL345 - Monitor ambulatorio de flujo glotico ===\r\n");

    /* --- Configuracion del sensor para la prueba ---
     * Rango +-2g, full-resolution (3.9 mg/LSB), ODR 25 Hz en modo
     * normal (sin low power ni FIFO), para poder leer muestra a
     * muestra y verificar timings/valores facilmente durante la
     * prueba. */
    adxl345_config_t config = {
        .range                  = ADXL345_RANGE_2G,
        .full_resolution        = true,
        .rate_code              = ADXL345_RATE_25HZ,
        .low_power_mode         = false,
        .use_fifo_stream        = false,
        .fifo_watermark_samples = 0,
    };

    adxl345_status_t status = adxl345_init(&config);
    switch (status) {
        case ADXL345_OK:
            printf("[OK] ADXL345 inicializado correctamente (DEVID verificado).\r\n");
            break;
        case ADXL345_ERROR_DEVID:
            printf("[ERROR] DEVID incorrecto: revisar cableado/soldadura del sensor.\r\n");
            return;
        case ADXL345_ERROR_SPI:
        default:
            printf("[ERROR] Fallo de comunicacion SPI durante la inicializacion.\r\n");
            return;
    }

    uint32_t sample_count = 0;

    while (true) {
        adxl345_accel_raw_t raw;
        status = adxl345_read_accel(&raw);

        if (status == ADXL345_OK) {
            int32_t x_mg = adxl345_raw_to_mg(raw.x, config.full_resolution, config.range);
            int32_t y_mg = adxl345_raw_to_mg(raw.y, config.full_resolution, config.range);
            int32_t z_mg = adxl345_raw_to_mg(raw.z, config.full_resolution, config.range);

            printf("Muestra %5" PRIu32 " | RAW X=%6d Y=%6d Z=%6d | mg X=%6" PRId32
                   " Y=%6" PRId32 " Z=%6" PRId32 "\r\n",
                   sample_count, raw.x, raw.y, raw.z, x_mg, y_mg, z_mg);
        } else {
            printf("[ERROR] Fallo al leer aceleracion (SPI).\r\n");
        }

        /* Verificacion adicional: nivel de ocupacion del FIFO (deberia
         * mantenerse cercano a 0 con use_fifo_stream = false, ya que
         * cada lectura vacia el registro de datos correspondiente). */
        uint8_t fifo_entries = 0;
        if (adxl345_get_fifo_entries(&fifo_entries) == ADXL345_OK) {
            printf("           FIFO entries = %u\r\n", fifo_entries);
        }

        sample_count++;

        /* --- Prueba de bajo consumo: standby / wakeup ---
         * Cada TEST_STANDBY_EVERY_N muestras, se fuerza al sensor a
         * standby y luego se lo despierta, para validar que el driver
         * de ahorro de energia funciona correctamente antes de un
         * despliegue de varios dias con bateria. */
        if ((sample_count % TEST_STANDBY_EVERY_N) == 0U) {
            printf(">>> Probando standby...\r\n");
            adxl345_standby();
            vTaskDelay(pdMS_TO_TICKS(500));

            printf(">>> Probando wakeup...\r\n");
            adxl345_wakeup();
            /* Datasheet: turn-on/wake-up time ~11 ms a 100Hz; a 25Hz
             * es mayor, se da margen antes de la proxima lectura. */
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(TEST_PERIOD_MS));
    }
}

/*==================[end of file]===============================================*/