/**
 * @file adxl345.c
 * @brief Implementacion del driver ADXL345 sobre la HAL spi_mcu.c.
 *
 * @details
 * Protocolo SPI del ADXL345 (4 hilos, modo 3: CPOL=1, CPHA=1):
 *
 *   Byte 0 enviado por el master:  [ R/W | MB | A5 A4 A3 A2 A1 A0 ]
 *     R/W = 1 -> lectura, 0 -> escritura
 *     MB  = 1 -> multiple-byte (rafaga), 0 -> un solo registro
 *     A5..A0 = direccion de registro (0x00 a 0x39)
 *
 *   Bytes siguientes: datos (uno por registro, en orden ascendente de
 *   direccion mientras el reloj siga activo y CS se mantenga bajo).
 *
 * El bus SPI del microcontrolador (driver ESP-IDF subyacente a spi_mcu.c)
 * gestiona automaticamente el pin CS durante la duracion de cada transaccion
 * (spi_device_transmit / spi_device_polling_transmit), por lo que el driver
 * ADXL345 NO necesita manipular CS manualmente: basta con enviar en una unica
 * llamada a SpiWrite/SpiReadWrite el byte de direccion seguido de los bytes
 * de datos, ya que ambos forman una sola transaccion SPI (CS permanece bajo
 * durante toda la operacion, tal como exige el datasheet).
 */

/*==================[inclusions]=============================================*/
#include "adxl345.h"
#include <string.h>

/*==================[internal data definition]===============================*/

/** @brief Guarda la configuracion activa para poder escalar lecturas a mg. */
static adxl345_config_t s_active_config;
static bool s_initialized = false;

/*==================[internal functions declaration]==========================*/
static adxl345_status_t adxl345_configure_bus(void);

/*==================[internal functions definition]============================*/

/**
 * @brief Inicializa el bus SPI (spi_mcu) para el dispositivo del ADXL345.
 *        Se usa modo POLLING por defecto: en un dispositivo de monitoreo
 *        ambulatorio la tasa de muestreo suele ser baja (<=100 Hz) y las
 *        transacciones son cortas, por lo que el polling evita la sobrecarga
 *        de una ISR y simplifica el manejo de energia (no hay callbacks
 *        pendientes que mantener activo al CPU).
 */
static adxl345_status_t adxl345_configure_bus(void)
{
    spi_mcu_config_t spi_cfg = {
        .device        = ADXL345_SPI_DEVICE,
        .bitrate       = ADXL345_SPI_BITRATE_HZ,
        .clk_mode      = ADXL345_SPI_CLK_MODE,
        .transfer_mode = SPI_POLLING,
        .func_p        = NULL,
        .param_p       = NULL,
    };

    if (SpiInit(&spi_cfg) != 0) {
        return ADXL345_ERROR_SPI;
    }
    return ADXL345_OK;
}

/*==================[external functions definition]============================*/

adxl345_status_t adxl345_write_register(uint8_t reg, uint8_t value)
{
    /* Una sola transaccion SPI: [addr|W][data]. CS bajo durante ambos bytes. */
    uint8_t tx_buf[2];
    tx_buf[0] = (reg & 0x3FU) | ADXL345_WRITE_BIT;
    tx_buf[1] = value;

    /* SpiWrite no necesita rx_buffer: los datos que el ADXL345 devuelve
     * durante una escritura no son significativos y se descartan. */
    SpiWrite(ADXL345_SPI_DEVICE, tx_buf, sizeof(tx_buf));

    return ADXL345_OK;
}

adxl345_status_t adxl345_read_registers(uint8_t reg, uint8_t *rx_buf, uint8_t length)
{
    if ((rx_buf == NULL) || (length == 0U)) {
        return ADXL345_ERROR_SPI;
    }

    /* Buffer de tx/rx de tamano length+1: el primer byte transmite la
     * direccion+flags; los siguientes `length` bytes son "dummy" en tx
     * (0x00) para generar los pulsos de reloj necesarios, y en rx
     * contendran los datos reales devueltos por el sensor (el dato
     * correspondiente al byte de direccion es descartado, ver Figura 38
     * del datasheet: el primer byte de MISO durante la fase de
     * direccionamiento no es valido). */
    uint8_t tx_buf[9];  /* 1 (addr) + hasta 6 datos (X0,X1,Y0,Y1,Z0,Z1) + margen */
    uint8_t rx_tmp[9];

    if ((length + 1U) > sizeof(tx_buf)) {
        return ADXL345_ERROR_SPI; /* longitud no soportada por el buffer estatico */
    }

    memset(tx_buf, 0x00, length + 1U);
    tx_buf[0] = (reg & 0x3FU) | ADXL345_READ_BIT |
                ((length > 1U) ? ADXL345_MULTIBYTE_BIT : 0x00U);

    SpiReadWrite(ADXL345_SPI_DEVICE, tx_buf, rx_tmp, length + 1U);

    /* rx_tmp[0] corresponde al byte de direccion (invalido); los datos
     * utiles arrancan en rx_tmp[1]. */
    memcpy(rx_buf, &rx_tmp[1], length);

    return ADXL345_OK;
}

adxl345_status_t adxl345_init(const adxl345_config_t *config)
{
    if (config == NULL) {
        return ADXL345_ERROR_SPI;
    }

    if (adxl345_configure_bus() != ADXL345_OK) {
        return ADXL345_ERROR_SPI;
    }

    /* --- Autoverificacion: leer DEVID y comparar con 0xE5 ---
     * Esto es critico en un dispositivo medico: detecta temprano un
     * sensor mal soldado, un cable desconectado, o un bus SPI mal
     * configurado, antes de iniciar una sesion de registro de varios dias. */
    uint8_t devid = 0;
    if (adxl345_read_registers(ADXL345_REG_DEVID, &devid, 1) != ADXL345_OK) {
        return ADXL345_ERROR_SPI;
    }
    if (devid != ADXL345_DEVID_VALUE) {
        return ADXL345_ERROR_DEVID;
    }

    /* --- Poner el sensor en standby antes de configurar ---
     * Recomendado por el datasheet: configurar en standby y luego pasar
     * a modo medicion, para evitar transitorios/ruido en las primeras
     * muestras. */
    adxl345_write_register(ADXL345_REG_POWER_CTL, 0x00U);

    /* --- DATA_FORMAT: rango, resolucion, SPI 4 hilos --- */
    uint8_t data_format = (config->range & ADXL345_RANGE_MASK);
    if (config->full_resolution) {
        data_format |= ADXL345_DATA_FORMAT_FULL_RES;
    }
    /* SPI_3WIRE bit se deja en 0 -> modo 4 hilos (SDI/SDO separados) */
    adxl345_write_register(ADXL345_REG_DATA_FORMAT, data_format);

    /* --- BW_RATE: ODR y modo de bajo consumo ---
     * Para monitoreo ambulatorio de movimiento de laringe/cuello, una
     * ODR de 25-100 Hz suele ser mas que suficiente y reduce
     * drasticamente el consumo (ver Tabla 7/8 del datasheet: 100 Hz
     * normal = 140 uA vs 12.5 Hz = 50 uA vs 12.5 Hz low-power = 34 uA). */
    uint8_t bw_rate = (config->rate_code & ADXL345_BW_RATE_MASK);
    if (config->low_power_mode) {
        bw_rate |= ADXL345_BW_RATE_LOW_POWER;
    }
    adxl345_write_register(ADXL345_REG_BW_RATE, bw_rate);

    /* --- FIFO: modo stream para minimizar transacciones SPI ---
     * Acumular varias muestras en el FIFO del ADXL345 y leerlas en un
     * solo burst reduce drasticamente la cantidad de veces que el MCU
     * debe salir de modo de bajo consumo para atender al sensor,
     * comparado con leer muestra a muestra a la ODR configurada. */
    if (config->use_fifo_stream) {
        uint8_t fifo_ctl = ADXL345_FIFO_MODE_STREAM |
                            (config->fifo_watermark_samples & ADXL345_FIFO_SAMPLES_MASK);
        adxl345_write_register(ADXL345_REG_FIFO_CTL, fifo_ctl);

        /* Habilitar interrupcion de watermark en INT1 para despertar al
         * MCU solo cuando haya suficientes muestras acumuladas. */
        adxl345_write_register(ADXL345_REG_INT_MAP, 0x00U); /* todo a INT1 */
        adxl345_write_register(ADXL345_REG_INT_ENABLE, ADXL345_INT_WATERMARK);
    } else {
        adxl345_write_register(ADXL345_REG_FIFO_CTL, ADXL345_FIFO_MODE_BYPASS);
    }

    /* --- Entrar en modo medicion --- */
    adxl345_write_register(ADXL345_REG_POWER_CTL, ADXL345_POWER_CTL_MEASURE);

    s_active_config = *config;
    s_initialized = true;

    return ADXL345_OK;
}

adxl345_status_t adxl345_read_accel(adxl345_accel_raw_t *out)
{
    if (out == NULL) {
        return ADXL345_ERROR_SPI;
    }

    /* Burst read de 6 bytes desde DATAX0: garantiza que X, Y, Z
     * pertenecen a la misma muestra (evita "tearing" entre ejes que
     * podria ocurrir si se leyeran registros individualmente mientras
     * el sensor sigue actualizando datos). */
    uint8_t raw[6];
    adxl345_status_t status = adxl345_read_registers(ADXL345_REG_DATAX0, raw, 6);
    if (status != ADXL345_OK) {
        return status;
    }

    /* Formato: LSB primero (DATAx0), luego MSB (DATAx1), complemento a 2. */
    out->x = (int16_t)((raw[1] << 8) | raw[0]);
    out->y = (int16_t)((raw[3] << 8) | raw[2]);
    out->z = (int16_t)((raw[5] << 8) | raw[4]);

    return ADXL345_OK;
}

int32_t adxl345_raw_to_mg(int16_t raw, bool full_resolution, uint8_t range)
{
    /* En modo full-resolution la escala es siempre ~3.9 mg/LSB,
     * independientemente del rango (Tabla 1 del datasheet). */
    if (full_resolution) {
        return ((int32_t)raw * 39) / 10; /* 3.9 mg/LSB */
    }

    /* En modo 10-bit fijo, la escala depende del rango seleccionado. */
    switch (range & ADXL345_RANGE_MASK) {
        case ADXL345_RANGE_2G:  return ((int32_t)raw * 39) / 10;   /* 3.9 mg/LSB */
        case ADXL345_RANGE_4G:  return ((int32_t)raw * 78) / 10;   /* 7.8 mg/LSB */
        case ADXL345_RANGE_8G:  return ((int32_t)raw * 156) / 10;  /* 15.6 mg/LSB */
        case ADXL345_RANGE_16G: return ((int32_t)raw * 312) / 10;  /* 31.2 mg/LSB */
        default:                return 0;
    }
}

adxl345_status_t adxl345_get_fifo_entries(uint8_t *entries)
{
    if (entries == NULL) {
        return ADXL345_ERROR_SPI;
    }

    uint8_t fifo_status = 0;
    adxl345_status_t status = adxl345_read_registers(ADXL345_REG_FIFO_STATUS, &fifo_status, 1);
    if (status != ADXL345_OK) {
        return status;
    }

    *entries = fifo_status & 0x3FU; /* bits D5:D0 = Entries */
    return ADXL345_OK;
}

adxl345_status_t adxl345_standby(void)
{
    /* Limpiar el bit MEASURE preserva el contenido del FIFO y baja el
     * consumo a ~0.1 uA tipico: ideal para pausas largas en el
     * monitoreo sin perder la configuracion del sensor. */
    return adxl345_write_register(ADXL345_REG_POWER_CTL, 0x00U);
}

adxl345_status_t adxl345_wakeup(void)
{
    if (!s_initialized) {
        return ADXL345_ERROR_SPI;
    }
    return adxl345_write_register(ADXL345_REG_POWER_CTL, ADXL345_POWER_CTL_MEASURE);
}

/*==================[end of file]===============================================*/
