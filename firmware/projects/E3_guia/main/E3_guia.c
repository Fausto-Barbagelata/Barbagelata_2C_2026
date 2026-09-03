/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "switch.h"

/*==================[macros and definitions]=================================*/
#define ON     1
#define OFF    2
#define TOGGLE 3

/*==================[internal data definition]===============================*/
//Esta definicion define un tipo de dato personalizado llamado my_leds basado en esa estructura
typedef struct leds
{
    uint8_t mode;
    uint8_t n_led;
    uint8_t n_ciclos;
    uint16_t periodo;
} my_leds;

/*==================[internal functions declaration]=========================*/
//declara el prototipo de la funcion
void controlar_leds(my_leds *led_config);//la funcion recibe la direccion de memoria de la estructura my_leds enviada en la variable led_config

/*==================[external functions definition]==========================*/

void controlar_leds(my_leds *led_config) {

    if (led_config->mode == ON) {//basicamente lo que hace es ver si el campo mode de la estructura apuntada por led_config es igual a ON
        if (led_config->n_led == 1) {
            LedOn(LED_1);
        } else if (led_config->n_led == 2) {
            LedOn(LED_2);
        } else if (led_config->n_led == 3) {
            LedOn(LED_3);
        }
    } 

    else if (led_config->mode == OFF) {
        if (led_config->n_led == 1) {
            LedOff(LED_1);
        } else if (led_config->n_led == 2) {
            LedOff(LED_2);
        } else if (led_config->n_led == 3) {
            LedOff(LED_3);
        }
    } 

    else if (led_config->mode == TOGGLE) {

        uint16_t retardo_pasos = led_config->periodo / 100; //calcula cuantos retardos de 100ms se necesitan dividiendo el periodo entre 100

        for (uint8_t i = 0; i < led_config->n_ciclos; i++) {

            if (led_config->n_led == 1) {
                LedToggle(LED_1);
            } else if (led_config->n_led == 2) {
                LedToggle(LED_2);
            } else if (led_config->n_led == 3) {
                LedToggle(LED_3);
            }

            for (uint16_t j = 0; j < retardo_pasos; j++) {
                vTaskDelay(100 / portTICK_PERIOD_MS);//convierte milisegundos en numero de ticks
            }
        }
    }
}

/*==================[main function]==========================================*/
void app_main(void) 
{
    LedsInit();
    SwitchesInit(); 

    my_leds config;
    config.mode = TOGGLE;   
    config.n_led = 2;       
    config.n_ciclos = 10;   
    config.periodo = 500; 

    controlar_leds(&config);
}
/*==================[end of file]============================================*/