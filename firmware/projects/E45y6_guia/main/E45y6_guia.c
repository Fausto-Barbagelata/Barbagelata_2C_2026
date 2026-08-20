/*! @mainpage Template
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */
/****************************************************************************** 
                            EJERCICIO 4 y 5
******************************************************************************/
/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_mcu.h"

/*==================[macros and definitions]=================================*/
#define maximo_de_digitos 5

/*==================[internal data definition]===============================*/
typedef struct
{
    gpio_t pin;
    io_t dir;
} gpioConf_t;
/*==================[internal functions declaration]=========================*/
int8_t convert_a_BCD(uint16_t data, uint8_t digits, uint8_t *bcd_number);
void BCD_a_GPIO(uint8_t bcd_digit, gpioConf_t *gpio_array);
/*==================[external functions definition]==========================*/
int8_t convert_a_BCD(uint16_t data, uint8_t digits, uint8_t *bcd_number)
{
    for (uint8_t i = 0; i < digits; i++) {
        bcd_number[i] = data % 10;
        data = data / 10;
    }
    return 0;
}

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
/*==================[main function]==========================================*/
void app_main(void) 
{
    gpioConf_t bcd_gpios[4] = {
        {GPIO_20, GPIO_OUTPUT},
        {GPIO_21, GPIO_OUTPUT},
        {GPIO_22, GPIO_OUTPUT},
        {GPIO_23, GPIO_OUTPUT} 
    };
    for (uint8_t i = 0; i < 4; i++) {
        GPIOInit(bcd_gpios[i].pin, bcd_gpios[i].dir);
    }

    uint16_t test_value = 5;
    uint8_t digits_count = 1;
    uint8_t bcd_output[maximo_de_digitos] = {0};

    convert_a_BCD(test_value, digits_count, bcd_output);
    BCD_a_GPIO(bcd_output[0], bcd_gpios);
}
/*==================[end of file]============================================*/
