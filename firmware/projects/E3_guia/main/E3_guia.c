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
typedef struct leds
{
    uint8_t mode;        // Modo de funcionamiento: ON, OFF o TOGGLE
    uint8_t n_led;       // Número de LED a controlar (1, 2 o 3)
    uint8_t n_ciclos;    // Cantidad de repeticiones (solo para modo TOGGLE)
    uint16_t periodo;    // Tiempo en milisegundos de cada ciclo
} my_leds;

/*==================[internal functions declaration]=========================*/
void controlar_leds(my_leds *led_config); // Esta declaracion la hacemos para evitar probleams de compilacion

/*==================[external functions definition]==========================*/

/**
 * @brief Función que controla los LEDs recibiendo un puntero a la estructura con los parámetros
 * @param led_config Puntero a la estructura de tipo my_leds
 */
void controlar_leds(my_leds *led_config) {
    
    // -------------------------------------------------------------------------
    // CASO 1: MODO ON (Encendido permanente)
    // -------------------------------------------------------------------------
    if (led_config->mode == ON) {
        if (led_config->n_led == 1) {
            LedOn(LED_1); // Enciende el LED 1
        } else if (led_config->n_led == 2) {
            LedOn(LED_2); // Enciende el LED 2
        } else if (led_config->n_led == 3) {
            LedOn(LED_3); // Enciende el LED 3
        }
    } 
    // -------------------------------------------------------------------------
    // CASO 2: MODO OFF (Apagado permanente)
    // -------------------------------------------------------------------------
    else if (led_config->mode == OFF) {
        if (led_config->n_led == 1) {
            LedOff(LED_1); // Apaga el LED 1
        } else if (led_config->n_led == 2) {
            LedOff(LED_2); // Apaga el LED 2
        } else if (led_config->n_led == 3) {
            LedOff(LED_3); // Apaga el LED 3
        }
    } 
    // -------------------------------------------------------------------------
    // CASO 3: MODO TOGGLE (Parpadeo por cantidad de ciclos y período determinado)
    // -------------------------------------------------------------------------
    else if (led_config->mode == TOGGLE) {
        
        // Calcula cuántas veces hay que hacer retardos de 100ms para juntar el 'periodo' completo.
        // Ejemplo: Si periodo = 500ms -> retardo_pasos = 500 / 100 = 5 pasos de 100ms.
        uint16_t retardo_pasos = led_config->periodo / 100;

        // Bucle externo: Cuenta la cantidad de ciclos (i va desde 0 hasta n_ciclos - 1)
        for (uint8_t i = 0; i < led_config->n_ciclos; i++) {
            
            // Cambia el estado del LED seleccionado (si está encendido lo apaga, y viceversa)
            if (led_config->n_led == 1) {
                LedToggle(LED_1);
            } else if (led_config->n_led == 2) {
                LedToggle(LED_2);
            } else if (led_config->n_led == 3) {
                LedToggle(LED_3);
            }

            // Bucle interno: Genera el tiempo de espera usando la función vTaskDelay con pasos de 100ms
            for (uint16_t j = 0; j < retardo_pasos; j++) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
    }
}

/*==================[main function]==========================================*/
void app_main(void) 
{
    // 1. Inicialización de los drivers de periféricos (LEDs y Switches)
    LedsInit();
    SwitchesInit(); 

    // 2. Creación y carga de la estructura de prueba
    my_leds config;
    config.mode = TOGGLE;   // Seleccionamos el modo TOGGLE
    config.n_led = 2;       // Indicamos que queremos controlar el LED 1
    config.n_ciclos = 10;   // Queremos 10 cambios de estado
    config.periodo = 500;   // Cada semi-ciclo durará 500 ms

    // 3. Llamada a la función pasando la dirección de memoria (&) de la estructura
    controlar_leds(&config);
}
/*==================[end of file]============================================*/