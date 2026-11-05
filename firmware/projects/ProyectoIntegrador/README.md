# Template
# Dosímetro de Voz y Analizador Vibracional Laríngeo (ESP32)

Sistema embebido en tiempo real para el monitoreo, adquisición y análisis de vibraciones laríngeas mediante acelerometría triaxial (ADXL345), filtros digitales IIR, estimación de frecuencia fundamental ($F_0$) y transmisión dual (UART / Bluetooth Low Energy). Desarrollado sobre **FreeRTOS** para la plataforma **ESP32** (ESP-IDF / Framework de Cátedra).

---

## Tabla de Contenidos
- [Características Principales](#-características-principales)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Hardware Requerido](#-hardware-requerido)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Flujo de Funcionamiento](#-flujo-de-funcionamiento)
- [Procesamiento Digital de Señales (DSP)](#-procesamiento-digital-de-señales-dsp)
- [Configuración y Parámetros](#-configuración-y-parámetros)
- [Uso e Interfaz](#-uso-e-interfaz)

---

## Características Principales

* **Adquisición a alta velocidad:** Muestreo determinístico a **1600 Hz** sincronizado mediante temporizador por hardware (Timer ISR).
* **Filtrado IIR en tiempo real:** Cadena de filtros digitales (Pasa-Altos a 60 Hz + Pasa-Bajos a 600 Hz) para aislar la componente fonatoria y eliminar gravedad y ruido de alta frecuencia.
* **Calibración Guiada de 3 Pasos:**
  1. Offset de gravedad estática (10 segundos).
  2. Nivel de ruido / silencio.
  3. Emisión de voz cómoda (frecuencia fundamental $F_0$) y grito.
* **Estimación de $F_0$ y Género Vocal:** Algoritmo en el dominio del tiempo con umbrado adaptativo para determinar la frecuencia fundamental en Hz y clasificar la voz (masculina/femenina/ambigua).
* **Monitoreo de Intensidad Vocal (RMS):** Estimación continua de valor eficaz con suavizado por constante de tiempo ($100\text{ ms}$).
* **Transmisión Dual:**
  * **UART:** 921,600 baudios (señal completa a 1600 Hz).
  * **Bluetooth Low Energy (BLE):** Submuestreo a ~100 Hz cifrado en formato compatible con la app *Bluetooth Electronics* (`*G<valor>*`).
* **Indicación por LEDs:** Retroalimentación visual inmediata según la intensidad (Verde = Suave, Amarillo = Normal, Rojo = Grito).

---

## Arquitectura del Sistema

El firmware utiliza una arquitectura multitarea determinística sobre **FreeRTOS**:

```
                  +--------------------------+
                  | Hardware Timer (1600 Hz) |
                  +------------+-------------+
                               | ISR (vTaskNotifyGiveFromISR)
                               v
                   +-----------------------+
                   |   AcquisitionTask     | Priority 6
                   |  (Lectura ADXL345)    |
                   +-----------+-----------+
                               | xTaskNotifyGive
                               v
                   +-----------------------+
                   |    ProcessingTask     | Priority 5
                   |  (IIR, RMS, F0, BLE)  |
                   +----+-------------+----+
                        |             |
           +------------+             +------------+
           v                                       v
  +------------------+                   +-------------------+
  |     LedTask      | Priority 3        |    SwitchTask     | Priority 3
  |  (Display LEDs)  |                   | (Lectura Botones) |
  +------------------+                   +-------------------+
```

---

## Hardware Requerido

1. **Placa de desarrollo ESP32** (o placa educativa basada en ESP32-C6 / EDU-ESP).
2. **Acelerómetro Triaxial ADXL345** (Conexión SPI/I2C).
3. **Pulsadores (Switches):**
   * `SWITCH_1`: Avanzar de fase / Confirmar paso de calibración.
   * `SWITCH_2`: Reintentar/Repetir fase de calibración.
4. **Barras de LEDs:**
   * `LED_1` (Verde): Nivel vocal bajo / suave.
   * `LED_2` (Amarillo): Nivel vocal normal.
   * `LED_3` (Rojo): Nivel vocal fuerte / grito.

---

## Estructura del Código

* `ProyectoIntegrador.c`: Código fuente principal que implementa el ciclo de vida, tareas de FreeRTOS y lógica DSP.
* **Drivers del sistema:**
  * `adxl345.h`: Controlador del acelerómetro.
  * `iir_filter.h`: Módulo de filtrado digital IIR.
  * `ble_mcu.h`: Abstracción de Bluetooth Low Energy.
  * `timer_mcu.h` / `uart_mcu.h`: Manejo de periféricos de temporización y puerto serie.

---

## Flujo de Funcionamiento

```
 [Inicio App] ──> [Calibración Gravedad (10s)] ──> [Paso 1: Silencio (10s)] 
                                                            │
                                                     (Press SWITCH_1)
                                                            ▼
 [Monitoreo LIVE] <── [Paso 3: Grito (5s)] <── [Paso 2: Voz Cómoda /a/ (5s)]
                        (Press SWITCH_1)           (Cálculo F0 + Género)
```

1. **Calibración Estática:** Mantiene el sensor en reposo durante 10 s para restar la aceleración de la gravedad ($g$).
2. **Test Guiado Vocal:**
   * **Silencio:** Registra el nivel base de ruido.
   * **Voz Cómoda:** Mide el RMS medio y captura las muestras para calcular la frecuencia fundamental ($F_0$).
   * **Grito:** Mide la aceleración máxima tolerada/alcanzada durante fonación intensa.
3. **Monitoreo en Vivo:** Muestra el nivel vocal en la barra de LEDs y transmite datos filtrados por UART/BLE.

---

## Procesamiento Digital de Señales (DSP)

### Filtrado Digital IIR
$$H(z) = H_{\text{HPF}}(z) \cdot H_{\text{LPF}}(z)$$
* **Filtro Pasa-Altos:** $f_c = 60\text{ Hz}$ (Orden 2).
* **Filtro Pasa-Bajos:** $f_c = 600\text{ Hz}$ (Orden 2).

### Estimación de Frecuencia Fundamental ($F_0$)
1. Identificación del pico máximo en la ventana almacenada.
2. Selección de picos por encima del umbral adaptativo ($\text{Max} \times 0.5$).
3. Promediado de períodos entre picos para calcular:
$$F_0 = \frac{f_{\text{muestreo}}}{\text{Período Promedio (muestras)}}$$

### Clasificación Tonal
| Rango de $F_0$ | Clasificación |
| :--- | :--- |
| **85.0 Hz - 180.0 Hz** | Voz Masculina |
| **165.0 Hz - 255.0 Hz** | Voz Femenina |
| **165.0 Hz - 180.0 Hz** | Rango Ambiguo / Solapado |

---

## Configuración y Parámetros

En `ProyectoIntegrador.c` se encuentran los macros principales para ajustar el comportamiento:

```c
#define UART_BAUD_RATE          921600
#define SAMPLE_FREQ_HZ          1600
#define BLE_DOWNSAMPLE_FACTOR   16
#define SELECTED_AXIS           AXIS_Z
```

---

## Uso e Interfaz

* **Monitor Serie (PC):** Configurar terminal a **921,600 baudios**. Observará la salida gráfica o texto tabulado de las aceleraciones en `mg`.
* **Aplicación Móvil (Bluetooth Electronics):**
  * Conectarse al dispositivo denominado **`DOSIMETRO_VOZ`**.
  * Configurar un gráfico para recibir la variable enviada en formato `*G<valor>*`.
```