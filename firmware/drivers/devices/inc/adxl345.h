/**
 * @file adxl345.h
 * @brief Driver para el acelerometro digital de 3 ejes ADXL345 (Analog Devices)
 *        sobre interfaz SPI de 4 hilos, usando la capa HAL spi_mcu.c/h.
 *
 * @details Pensado para un monitor ambulatorio de flujo glotico (uso continuo
 *          de varios dias con bateria). Se priorizan lecturas en rafaga (burst),
 *          uso de FIFO y modos de baja potencia para minimizar transacciones SPI
 *          y consumo de energia.
 *
 * Referencia: ADXL345 Data Sheet Rev. D (Analog Devices).
 */

#ifndef ADXL345_H
#define ADXL345_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include <stdbool.h>
#include "spi_mcu.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros: SPI device / physical mapping]==================*/

/** @brief Dispositivo SPI (CS) al que esta conectado el ADXL345.
 *  Ajustar segun el diseno de PCB (SPI_1, SPI_2 o SPI_3 definidos en spi_mcu.h).
 */
#define ADXL345_SPI_DEVICE         SPI_1

/** @brief Velocidad de reloj SPI. Datasheet: maximo 5 MHz (100 pF de carga).
 *  Se usa un valor conservador para mejorar la integridad de senal en cables
 *  largos hacia el sensor pegado al cuello del paciente.
 */
#define ADXL345_SPI_BITRATE_HZ     2000000UL

/** @brief Modo de reloj SPI requerido por el ADXL345: CPOL = 1, CPHA = 1 (modo 3). */
#define ADXL345_SPI_CLK_MODE       3

/*==================[macros: protocolo SPI del ADXL345]======================*/

#define ADXL345_READ_BIT            0x80U   /*!< Bit R/W: 1 = lectura */
#define ADXL345_WRITE_BIT           0x00U   /*!< Bit R/W: 0 = escritura */
#define ADXL345_MULTIBYTE_BIT       0x40U   /*!< Bit MB: habilita lectura/escritura en rafaga */

/*==================[macros: mapa de registros]==============================*/

#define ADXL345_REG_DEVID           0x00U   /*!< ID de dispositivo (RO) */
#define ADXL345_REG_THRESH_TAP      0x1DU   /*!< Umbral de tap */
#define ADXL345_REG_OFSX            0x1EU   /*!< Offset eje X */
#define ADXL345_REG_OFSY            0x1FU   /*!< Offset eje Y */
#define ADXL345_REG_OFSZ            0x20U   /*!< Offset eje Z */
#define ADXL345_REG_DUR             0x21U   /*!< Duracion maxima de tap */
#define ADXL345_REG_LATENT          0x22U   /*!< Latencia entre taps */
#define ADXL345_REG_WINDOW          0x23U   /*!< Ventana para doble tap */
#define ADXL345_REG_THRESH_ACT      0x24U   /*!< Umbral de actividad */
#define ADXL345_REG_THRESH_INACT    0x25U   /*!< Umbral de inactividad */
#define ADXL345_REG_TIME_INACT      0x26U   /*!< Tiempo de inactividad */
#define ADXL345_REG_ACT_INACT_CTL   0x27U   /*!< Control de ejes para act/inact */
#define ADXL345_REG_THRESH_FF       0x28U   /*!< Umbral de caida libre */
#define ADXL345_REG_TIME_FF         0x29U   /*!< Tiempo minimo de caida libre */
#define ADXL345_REG_TAP_AXES        0x2AU   /*!< Control de ejes para tap */
#define ADXL345_REG_ACT_TAP_STATUS  0x2BU   /*!< Estado de fuente de tap/actividad (RO) */
#define ADXL345_REG_BW_RATE         0x2CU   /*!< Data rate y modo de potencia */
#define ADXL345_REG_POWER_CTL       0x2DU   /*!< Control de ahorro de energia */
#define ADXL345_REG_INT_ENABLE      0x2EU   /*!< Habilitacion de interrupciones */
#define ADXL345_REG_INT_MAP         0x2FU   /*!< Mapeo de interrupciones a INT1/INT2 */
#define ADXL345_REG_INT_SOURCE      0x30U   /*!< Fuente de interrupciones (RO) */
#define ADXL345_REG_DATA_FORMAT     0x31U   /*!< Formato de datos de salida */
#define ADXL345_REG_DATAX0          0x32U   /*!< X LSB (RO) */
#define ADXL345_REG_DATAX1          0x33U   /*!< X MSB (RO) */
#define ADXL345_REG_DATAY0          0x34U   /*!< Y LSB (RO) */
#define ADXL345_REG_DATAY1          0x35U   /*!< Y MSB (RO) */
#define ADXL345_REG_DATAZ0          0x36U   /*!< Z LSB (RO) */
#define ADXL345_REG_DATAZ1          0x37U   /*!< Z MSB (RO) */
#define ADXL345_REG_FIFO_CTL        0x38U   /*!< Control de FIFO */
#define ADXL345_REG_FIFO_STATUS     0x39U   /*!< Estado de FIFO (RO) */

/** @brief Valor esperado del registro DEVID (345 en octal = 0xE5). */
#define ADXL345_DEVID_VALUE         0xE5U

/*==================[macros: bits de POWER_CTL (0x2D)]========================*/
#define ADXL345_POWER_CTL_LINK       (1U << 5)
#define ADXL345_POWER_CTL_AUTO_SLEEP (1U << 4)
#define ADXL345_POWER_CTL_MEASURE    (1U << 3)
#define ADXL345_POWER_CTL_SLEEP      (1U << 2)
#define ADXL345_POWER_CTL_WAKEUP_MASK 0x03U /*!< 00=8Hz 01=4Hz 10=2Hz 11=1Hz en sleep mode */

/*==================[macros: bits de BW_RATE (0x2C)]===========================*/
#define ADXL345_BW_RATE_LOW_POWER    (1U << 4)
#define ADXL345_BW_RATE_MASK         0x0FU

/** @brief Codigos de ODR (Output Data Rate) mas utiles para monitoreo ambulatorio. */
#define ADXL345_RATE_800HZ           0x0DU
#define ADXL345_RATE_400HZ           0x0CU
#define ADXL345_RATE_200HZ           0x0BU
#define ADXL345_RATE_100HZ           0x0AU /*!< Valor por defecto de fabrica */
#define ADXL345_RATE_50HZ            0x09U
#define ADXL345_RATE_25HZ            0x08U
#define ADXL345_RATE_12_5HZ          0x07U
#define ADXL345_RATE_6_25HZ          0x06U
#define ADXL345_RATE_3_13HZ          0x05U
#define ADXL345_RATE_1_56HZ          0x04U

/*==================[macros: bits de DATA_FORMAT (0x31)]======================*/
#define ADXL345_DATA_FORMAT_SELF_TEST  (1U << 7)
#define ADXL345_DATA_FORMAT_SPI_3WIRE  (1U << 6) /*!< 1 = 3 hilos, 0 = 4 hilos */
#define ADXL345_DATA_FORMAT_INT_INVERT (1U << 5)
#define ADXL345_DATA_FORMAT_FULL_RES   (1U << 3)
#define ADXL345_DATA_FORMAT_JUSTIFY    (1U << 2) /*!< 1 = MSB justificado, 0 = LSB con signo */

#define ADXL345_RANGE_2G             0x00U
#define ADXL345_RANGE_4G             0x01U
#define ADXL345_RANGE_8G             0x02U
#define ADXL345_RANGE_16G            0x03U
#define ADXL345_RANGE_MASK           0x03U

/*==================[macros: bits de FIFO_CTL (0x38)]=========================*/
#define ADXL345_FIFO_MODE_BYPASS     0x00U
#define ADXL345_FIFO_MODE_FIFO       0x40U
#define ADXL345_FIFO_MODE_STREAM     0x80U
#define ADXL345_FIFO_MODE_TRIGGER    0xC0U
#define ADXL345_FIFO_MODE_MASK       0xC0U
#define ADXL345_FIFO_TRIGGER_BIT     (1U << 5)
#define ADXL345_FIFO_SAMPLES_MASK    0x1FU  /*!< 0..31 muestras de watermark */

/*==================[macros: bits de INT_ENABLE / INT_MAP / INT_SOURCE]=======*/
#define ADXL345_INT_DATA_READY       (1U << 7)
#define ADXL345_INT_SINGLE_TAP       (1U << 6)
#define ADXL345_INT_DOUBLE_TAP       (1U << 5)
#define ADXL345_INT_ACTIVITY         (1U << 4)
#define ADXL345_INT_INACTIVITY       (1U << 3)
#define ADXL345_INT_FREE_FALL        (1U << 2)
#define ADXL345_INT_WATERMARK        (1U << 1)
#define ADXL345_INT_OVERRUN          (1U << 0)

/*==================[typedef]==================================================*/

/** @brief Codigos de retorno del driver. */
typedef enum {
    ADXL345_OK = 0,
    ADXL345_ERROR_SPI,          /*!< Fallo en la capa SPI */
    ADXL345_ERROR_DEVID,        /*!< DEVID leido no coincide (sensor no responde / mal cableado) */
} adxl345_status_t;

/** @brief Muestra de aceleracion cruda (cuentas, twos complement de 16 bits). */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} adxl345_accel_raw_t;

/** @brief Configuracion de alto nivel para adxl345_init(). */
typedef struct {
    uint8_t range;          /*!< ADXL345_RANGE_2G/4G/8G/16G */
    bool    full_resolution; /*!< true: full-res (4 mg/LSB en todos los rangos) */
    uint8_t rate_code;      /*!< ADXL345_RATE_xxx (ver BW_RATE) */
    bool    low_power_mode; /*!< true: activa LOW_POWER bit (mas ruido, menos consumo) */
    bool    use_fifo_stream; /*!< true: configura FIFO en modo stream */
    uint8_t fifo_watermark_samples; /*!< 0..31, umbral de interrupcion de watermark */
} adxl345_config_t;

/*==================[external functions declaration]==========================*/

/**
 * @brief Inicializa el bus SPI (via spi_mcu) y el ADXL345: verifica DEVID,
 *        configura DATA_FORMAT, BW_RATE, FIFO (opcional) y entra en modo medicion.
 *
 * @param config Puntero a configuracion deseada (rango, ODR, bajo consumo, FIFO).
 * @return ADXL345_OK si la inicializacion y la autoverificacion de DEVID fueron exitosas.
 */
adxl345_status_t adxl345_init(const adxl345_config_t *config);

/**
 * @brief Escribe un unico registro del ADXL345.
 * @param reg   Direccion de registro (0x00 - 0x39).
 * @param value Valor a escribir.
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_write_register(uint8_t reg, uint8_t value);

/**
 * @brief Lee uno o mas registros consecutivos del ADXL345 (lectura en rafaga).
 * @param reg      Direccion de registro inicial.
 * @param rx_buf   Buffer destino, debe tener al menos `length` bytes.
 * @param length   Cantidad de registros/bytes a leer.
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_read_registers(uint8_t reg, uint8_t *rx_buf, uint8_t length);

/**
 * @brief Lee una muestra de aceleracion (X, Y, Z) mediante burst read de 6 bytes
 *        desde DATAX0. Si el FIFO esta habilitado, esto extrae la muestra mas
 *        antigua disponible.
 *
 * @param out Puntero a estructura donde se almacenan los valores crudos.
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_read_accel(adxl345_accel_raw_t *out);

/**
 * @brief Convierte una lectura cruda a mili-g, segun el rango y modo de
 *        resolucion configurados.
 *
 * @param raw             Valor crudo de un eje (twos complement).
 * @param full_resolution true si el sensor esta en modo full-resolution.
 * @param range           Rango configurado (ADXL345_RANGE_xxx), solo relevante
 *                         si full_resolution es false.
 * @return Aceleracion en mili-g (mg).
 */
int32_t adxl345_raw_to_mg(int16_t raw, bool full_resolution, uint8_t range);

/**
 * @brief Devuelve la cantidad de muestras actualmente almacenadas en el FIFO.
 * @param entries Puntero de salida (0-32).
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_get_fifo_entries(uint8_t *entries);

/**
 * @brief Pone al sensor en modo standby (bajo consumo, 0.1 uA tipico),
 *        conservando el contenido del FIFO. Util para pausar el registro
 *        sin perder configuracion.
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_standby(void);

/**
 * @brief Vuelve a modo medicion luego de un adxl345_standby().
 * @return ADXL345_OK o ADXL345_ERROR_SPI.
 */
adxl345_status_t adxl345_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* ADXL345_H */
