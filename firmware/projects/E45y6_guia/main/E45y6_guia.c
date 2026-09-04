/*! @mainpage Ejercicios 4, 5 y 6 - Conversión BCD y control de display LCD
 *
 * @section genDesc General Description
 *
 * El programa permite representar un número decimal mediante su conversión
 * a código BCD (Binary Coded Decimal) y posteriormente enviar cada dígito
 * codificado a un conjunto de cuatro GPIO del ESP32.
 *
 * El sistema utiliza cuatro líneas GPIO para representar los cuatro bits
 * correspondientes a cada dígito BCD y tres líneas adicionales para
 * seleccionar los dígitos del display.
 *
 * El procedimiento general del programa es:
 * - Recibir un número decimal.
 * - Separar el número en sus diferentes dígitos decimales.
 * - Convertir cada dígito a su representación BCD de 4 bits.
 * - Enviar los cuatro bits del dígito a los GPIO correspondientes.
 * - Seleccionar mediante GPIO el dígito del display que se desea activar.
 *
 * En el ejemplo implementado se utiliza el número 285, por lo que se
 * obtienen los dígitos 2, 8 y 5, que son enviados secuencialmente al
 * display mediante las líneas de datos BCD y las líneas de selección.
 *
 * @section hardConn Hardware Connection
 *
 * Conexión de las líneas BCD:
 *
 * | Línea BCD | ESP32 GPIO | Descripción |
 * |:---------:|:----------:|:------------|
 * | BCD0      | GPIO_20    | Bit menos significativo del BCD |
 * | BCD1      | GPIO_21    | Segundo bit del BCD |
 * | BCD2      | GPIO_22    | Tercer bit del BCD |
 * | BCD3      | GPIO_23    | Bit más significativo del BCD |
 *
 * Conexión de selección de dígitos:
 *
 * | Dígito | ESP32 GPIO | Descripción |
 * |:------:|:----------:|:------------|
 * | 1      | GPIO_19    | Selección del primer dígito |
 * | 2      | GPIO_18    | Selección del segundo dígito |
 * | 3      | GPIO_9     | Selección del tercer dígito |
 *
 * Todos los GPIO utilizados son configurados como salidas digitales.
 *
 * @section software Software Description
 *
 * La función `convert_a_BCD()` separa el número decimal en sus diferentes
 * dígitos mediante divisiones sucesivas por 10 y almacena cada dígito en
 * un arreglo.
 *
 * La función `BCD_a_GPIO()` toma un dígito decimal codificado en BCD y
 * establece el estado lógico de cuatro GPIO según los bits correspondientes.
 *
 * Finalmente, `mostrarNumeroLCD()` combina ambas operaciones y controla
 * las líneas de selección de los dígitos del display.
 *
 * @section example Example
 *
 * Para el siguiente ejemplo:
 *
 * @code
 * uint16_t test_value = 285;
 * uint8_t digits_count = 3;
 * mostrarNumeroLCD(test_value, digits_count, bcd_gpios, lcd_select);
 * @endcode
 *
 * El número se separa en los dígitos:
 *
 * @code
 * 2 - 8 - 5
 * @endcode
 *
 * Cada dígito se representa mediante cuatro bits BCD:
 *
 * | Dígito decimal | BCD |
 * |:--------------:|:---:|
 * | 0              | 0000 |
 * | 1              | 0001 |
 * | 2              | 0010 |
 * | 3              | 0011 |
 * | 4              | 0100 |
 * | 5              | 0101 |
 * | 6              | 0110 |
 * | 7              | 0111 |
 * | 8              | 1000 |
 * | 9              | 1001 |
 *
 * @section changelog Changelog
 *
 * | Date       | Description |
 * |:----------:|:------------|
 * | 29/08/2026 | Documentación inicial en formato Doxygen |
 *
 * @author Fausto Benjamin Barbagelata (faustobenjaminbarbagelata@gmail.com)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_mcu.h"

/*==================[macros and definitions]=================================*/

/**
 * @brief Cantidad máxima de dígitos que puede almacenar el arreglo utilizado
 *        para la conversión del número.
 */
#define maximo_de_digitos 5

/*==================[internal data definition]===============================*/

/**
 * @brief Estructura de configuración de un GPIO.
 *
 * Esta estructura permite asociar un pin GPIO con su dirección de
 * funcionamiento.
 */
typedef struct
{
    gpio_t pin; /**< Pin GPIO utilizado. */
    io_t dir;   /**< Dirección del GPIO (entrada o salida). */
} gpioConf_t;

/*==================[internal functions declaration]=========================*/

/**
 * @brief Convierte un número decimal en sus dígitos individuales.
 *
 * Separa el número decimal mediante divisiones sucesivas por 10 y almacena
 * cada dígito en el arreglo recibido. Los dígitos se almacenan comenzando
 * por el menos significativo.
 *
 * @param[in] data Número decimal que se desea convertir.
 * @param[in] digits Cantidad de dígitos que se desean obtener.
 * @param[out] bcd_number Arreglo donde se almacenan los dígitos obtenidos.
 *
 * @return 0 La conversión se realizó correctamente.
 */
int8_t convert_a_BCD(uint16_t data, uint8_t digits, uint8_t *bcd_number);

/**
 * @brief Envía un dígito BCD a las cuatro líneas GPIO.
 *
 * Analiza los cuatro bits menos significativos del valor recibido y
 * configura los cuatro GPIO correspondientes según el valor de cada bit.
 *
 * @param[in] bcd_digit Dígito decimal codificado en BCD.
 * @param[in] gpio_array Arreglo de cuatro estructuras con la configuración
 *                       de los GPIO utilizados para transmitir el BCD.
 */
void BCD_a_GPIO(uint8_t bcd_digit, gpioConf_t *gpio_array);

/**
 * @brief Muestra un número utilizando las líneas BCD y selección de dígitos.
 *
 * Convierte el número recibido en sus dígitos decimales y posteriormente
 * envía cada dígito en formato BCD a los GPIO correspondientes. Además,
 * activa y desactiva la línea de selección asociada a cada posición.
 *
 * @param[in] data Número decimal que se desea mostrar.
 * @param[in] digits Cantidad de dígitos del número.
 * @param[in] bcd_gpios Arreglo de GPIO utilizados para transmitir el código BCD.
 * @param[in] lcd_select Arreglo de GPIO utilizados para seleccionar los dígitos.
 */
void mostrarNumeroLCD(uint32_t data, uint8_t digits, gpioConf_t *bcd_gpios, gpioConf_t *lcd_select);

/*==================[external functions definition]==========================*/

/**
 * @brief Convierte un número decimal en sus dígitos individuales.
 *
 * La función obtiene cada dígito mediante el resto de la división por 10.
 * Luego, el número se divide por 10 para continuar con el siguiente dígito.
 *
 * Por ejemplo, para el valor 285:
 *
 * @code
 * bcd_number[0] = 5;
 * bcd_number[1] = 8;
 * bcd_number[2] = 2;
 * @endcode
 *
 * @param[in] data Número decimal que se desea convertir.
 * @param[in] digits Cantidad de dígitos a obtener.
 * @param[out] bcd_number Arreglo donde se almacenan los dígitos.
 *
 * @return 0 La conversión se realizó correctamente.
 */
int8_t convert_a_BCD(uint16_t data, uint8_t digits, uint8_t *bcd_number)
{
    for (uint8_t i = 0; i < digits; i++) {
        bcd_number[i] = data % 10;
        data = data / 10;
    }

    return 0;
}

/**
 * @brief Convierte un dígito decimal a sus cuatro bits BCD y los envía a GPIO.
 *
 * Cada bit del dígito BCD es analizado individualmente. Si el bit vale 1,
 * se activa el GPIO correspondiente mediante GPIOOn(). Si vale 0, se
 * desactiva mediante GPIOOff().
 *
 * @param[in] bcd_digit Dígito decimal representado en BCD.
 * @param[in] gpio_array Arreglo de cuatro GPIO utilizados para representar
 *                       los cuatro bits del BCD.
 */
void BCD_a_GPIO(uint8_t bcd_digit, gpioConf_t *gpio_array)
{
    for (uint8_t b = 0; b < 4; b++) {
        uint8_t bit_value = (bcd_digit >> b) & 0x01;

        if (bit_value == 1) {
            GPIOOn(gpio_array[b].pin);
        } else {
            GPIOOff(gpio_array[b].pin);
        }
    }
}

/**
 * @brief Envía un número decimal al display mediante BCD.
 *
 * La función convierte primero el número recibido en sus dígitos
 * individuales. Luego recorre los dígitos desde el más significativo
 * hasta el menos significativo.
 *
 * Para cada dígito:
 * - Se convierte su valor a las cuatro líneas BCD.
 * - Se activa la línea de selección correspondiente.
 * - Se desactiva posteriormente dicha línea.
 *
 * @param[in] data Número decimal que se desea mostrar.
 * @param[in] digits Cantidad de dígitos que posee el número.
 * @param[in] bcd_gpios Arreglo de cuatro GPIO correspondientes a las
 *                       líneas BCD.
 * @param[in] lcd_select Arreglo de GPIO utilizados para seleccionar
 *                       cada posición del display.
 */
void mostrarNumeroLCD(uint32_t data, uint8_t digits, gpioConf_t *bcd_gpios, gpioConf_t *lcd_select)
{
    uint8_t digitos[maximo_de_digitos] = {0};

    convert_a_BCD(data, digits, digitos);

    for (int8_t i = digits - 1; i >= 0; i--) {
        BCD_a_GPIO(digitos[i], bcd_gpios);

        GPIOOn(lcd_select[digits - 1 - i].pin);
        GPIOOff(lcd_select[digits - 1 - i].pin);
    }
}

/*==================[main function]==========================================*/

/**
 * @brief Punto de entrada principal de la aplicación.
 *
 * Configura los GPIO utilizados para transmitir los cuatro bits BCD y
 * seleccionar los tres dígitos del display.
 *
 * Como prueba, se utiliza el valor 285 con tres dígitos y se llama a
 * mostrarNumeroLCD() para enviarlo al display.
 */
void app_main(void)
{
    /**
     * @brief GPIO utilizados para transmitir los cuatro bits BCD.
     *
     * GPIO_20 corresponde al bit 0, GPIO_21 al bit 1,
     * GPIO_22 al bit 2 y GPIO_23 al bit 3.
     */
    gpioConf_t bcd_gpios[4] = {
        {GPIO_20, GPIO_OUTPUT},
        {GPIO_21, GPIO_OUTPUT},
        {GPIO_22, GPIO_OUTPUT},
        {GPIO_23, GPIO_OUTPUT}
    };

    /**
     * @brief GPIO utilizados para seleccionar los tres dígitos del display.
     */
    gpioConf_t lcd_select[3] = {
        {GPIO_19, GPIO_OUTPUT},
        {GPIO_18, GPIO_OUTPUT},
        {GPIO_9,  GPIO_OUTPUT}
    };

    /* Inicialización de los GPIO correspondientes a las líneas BCD. */
    for (uint8_t i = 0; i < 4; i++) {
        GPIOInit(bcd_gpios[i].pin, bcd_gpios[i].dir);
    }

    /* Inicialización de los GPIO correspondientes a la selección de dígitos. */
    for (uint8_t i = 0; i < 3; i++) {
        GPIOInit(lcd_select[i].pin, lcd_select[i].dir);
    }

    /**
     * @brief Valor utilizado como prueba.
     */
    uint16_t test_value = 285;

    /**
     * @brief Cantidad de dígitos del valor de prueba.
     */
    uint8_t digits_count = 3;

    /* Envío del número al display. */
    mostrarNumeroLCD(test_value, digits_count, bcd_gpios, lcd_select);
}

/*==================[end of file]============================================*/