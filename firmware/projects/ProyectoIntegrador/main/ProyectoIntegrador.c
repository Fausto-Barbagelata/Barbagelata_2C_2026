/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "adxl345.h"
#include "iir_filter.h"
#include "led.h"
#include "switch.h"

/*==================[macros and definitions]=================================*/
#define UART_BAUD_RATE          921600
#define SAMPLE_FREQ_HZ          1600
#define SAMPLE_PERIOD_US        (1000000 / SAMPLE_FREQ_HZ)

/** @brief Duracion de la calibracion inicial de offset (gravedad) en segundos */
#define CALIBRATION_TIME_SEC    10
/** @brief Cantidad total de muestras para la calibracion de offset (16000 muestras) */
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

/** @brief Estructura de datos de una muestra de aceleracion en mg */
typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

/** @brief Duraciones de cada paso del test guiado de voz, en segundos */
#define VOICE_TEST_SILENCE_SEC      10
#define VOICE_TEST_COMFORTABLE_SEC  5
#define VOICE_TEST_SHOUT_SEC        5

#define VOICE_TEST_SILENCE_SAMPLES     (SAMPLE_FREQ_HZ * VOICE_TEST_SILENCE_SEC)
#define VOICE_TEST_COMFORTABLE_SAMPLES (SAMPLE_FREQ_HZ * VOICE_TEST_COMFORTABLE_SEC)
#define VOICE_TEST_SHOUT_SAMPLES       (SAMPLE_FREQ_HZ * VOICE_TEST_SHOUT_SEC)

/** @brief Constante de tiempo (s) del suavizado de RMS en el monitoreo en vivo */
#define LIVE_RMS_TIME_CONSTANT_SEC   0.1f

/** @brief Fraccion del rango [silencio, normal] reservada como margen de
 *  ruido antes de considerar que ya hay voz "bajita" presente. */
#define SILENCE_MARGIN_FRACTION   0.2f

/** @brief Umbral de deteccion de pico para F0, como fraccion de la
 *  amplitud maxima absoluta del buffer de voz comoda. */
#define PEAK_THRESHOLD_FRACTION   0.5f

/** @brief F0 maxima plausible para una vocal sostenida (Hz): define la
 *  distancia minima entre dos picos consecutivos aceptados. */
#define MAX_PLAUSIBLE_F0_HZ       350.0f

/** @brief Pasos del test guiado de voz */
typedef enum {
    VOICE_TEST_PHASE_SILENCE = 0,
    VOICE_TEST_PHASE_COMFORTABLE,
    VOICE_TEST_PHASE_SHOUT,
    VOICE_TEST_PHASE_LIVE,
} voice_test_phase_t;

/** @brief Categorias de nivel de voz */
typedef enum {
    VOICE_LEVEL_SILENCE = 0,
    VOICE_LEVEL_SOFT,
    VOICE_LEVEL_NORMAL,
    VOICE_LEVEL_LOUD,
} voice_level_t;

/** @brief Resultado de clasificacion de genero segun F0 */
typedef enum {
    VOICE_GENDER_UNDETERMINED = 0,
    VOICE_GENDER_MALE,
    VOICE_GENDER_FEMALE,
    VOICE_GENDER_AMBIGUOUS,
} voice_gender_t;

/** @brief Acumulador de RMS por bloque (ventana de N muestras conocida) */
typedef struct {
    double   sum_sq;
    uint32_t count;
} rms_block_t;

/** @brief Estimador de RMS corriente (promedio exponencial, sin buffer) */
typedef struct {
    float mean_sq;
    float alpha;
} rms_running_t;

/** @brief Umbrales de clasificacion de nivel de voz, calculados a partir
 *  de los 3 niveles RMS de referencia del test guiado. */
typedef struct {
    float threshold_silence_to_soft;
    float threshold_soft_to_normal;
    float threshold_normal_to_loud;
    bool  calibrated;
} voice_level_classifier_t;

/*==================[internal data definition]===============================*/
TaskHandle_t sample_task_handle = NULL;

/* Estado de calibracion de offset DC (gravedad) */
float offset_x = 0.0f;
float offset_y = 0.0f;
float offset_z = 0.0f;
bool is_calibrated = false;

double calib_sum_x = 0.0;
double calib_sum_y = 0.0;
double calib_sum_z = 0.0;
uint32_t calib_count = 0;

/* Estado del test guiado de voz */
voice_test_phase_t voice_test_phase = VOICE_TEST_PHASE_SILENCE;
uint32_t voice_test_sample_count = 0;
rms_block_t voice_test_rms_accum;

float level_silence_mg = 0.0f;
float level_normal_mg  = 0.0f;
float level_loud_mg    = 0.0f;

voice_level_classifier_t voice_classifier;
voice_level_t last_voice_level = VOICE_LEVEL_SILENCE;

/* Buffer de la fase de voz comoda, usado para estimar F0 por pico-a-pico */
float f0_buffer[VOICE_TEST_COMFORTABLE_SAMPLES];
float detected_f0_hz = 0.0f;
voice_gender_t detected_gender = VOICE_GENDER_UNDETERMINED;

/* Estimador de nivel RMS en vivo (una vez terminado el test guiado) */
rms_running_t live_rms_meter;

/*==================[internal functions definition]==========================*/

/*---------- Utilidades de nivel RMS (sin conocimiento de voz/sensor) ------*/

/** @brief Reinicia el acumulador de RMS por bloque. */
static void RmsBlockReset(rms_block_t *meter){
    meter->sum_sq = 0.0;
    meter->count = 0;
}

/** @brief Agrega una muestra al acumulador de RMS por bloque. */
static void RmsBlockAddSample(rms_block_t *meter, float sample){
    meter->sum_sq += (double)sample * (double)sample;
    meter->count++;
}

/** @brief Calcula el RMS de todas las muestras acumuladas (0 si no hay ninguna). */
static float RmsBlockGet(const rms_block_t *meter){
    if(meter->count == 0){
        return 0.0f;
    }
    return (float)sqrt(meter->sum_sq / (double)meter->count);
}

/** @brief Inicializa el estimador de RMS corriente (promedio exponencial). */
static void RmsRunningInit(rms_running_t *meter, float sample_rate_hz, float time_constant_s){
    float dt = 1.0f / sample_rate_hz;
    meter->alpha = dt / (time_constant_s + dt);
    meter->mean_sq = 0.0f;
}

/** @brief Actualiza el estimador de RMS corriente con una nueva muestra. */
static float RmsRunningUpdate(rms_running_t *meter, float sample){
    float sq = sample * sample;
    meter->mean_sq += meter->alpha * (sq - meter->mean_sq);
    return sqrtf(meter->mean_sq);
}

/*---------- Clasificador de nivel de voz (sin conocimiento de voz/sensor) --*/

/**
 * @brief Calcula los umbrales de clasificacion a partir de los 3 niveles
 * RMS de referencia medidos en el test guiado. El umbral silencio/bajito
 * se fija con un margen de ruido por encima del nivel de silencio; el
 * umbral bajito/normal es el punto medio entre silencio y normal; el
 * umbral normal/gritando es el punto medio entre normal y grito.
 */
static void VoiceLevelClassifierCalibrate(voice_level_classifier_t *clf,
                                           float level_silence,
                                           float level_normal,
                                           float level_loud){
    float range_silence_normal = level_normal - level_silence;
    float range_normal_loud    = level_loud - level_normal;

    clf->threshold_silence_to_soft = level_silence + SILENCE_MARGIN_FRACTION * range_silence_normal;
    clf->threshold_soft_to_normal  = level_silence + 0.5f * range_silence_normal;
    clf->threshold_normal_to_loud  = level_normal + 0.5f * range_normal_loud;
    clf->calibrated = true;
}

/** @brief Clasifica un valor RMS segun los umbrales ya calibrados. */
static voice_level_t VoiceLevelClassify(const voice_level_classifier_t *clf, float rms_value){
    if(!clf->calibrated){
        return VOICE_LEVEL_SILENCE;
    }
    if(rms_value < clf->threshold_silence_to_soft){
        return VOICE_LEVEL_SILENCE;
    }
    if(rms_value < clf->threshold_soft_to_normal){
        return VOICE_LEVEL_SOFT;
    }
    if(rms_value < clf->threshold_normal_to_loud){
        return VOICE_LEVEL_NORMAL;
    }
    return VOICE_LEVEL_LOUD;
}

/*---------- Estimador de F0 por pico-a-pico (sin conocimiento de voz/sensor) */

/**
 * @brief Estima la frecuencia fundamental (F0) de una señal periodica,
 * midiendo la distancia promedio entre picos consecutivos. Detecta
 * maximos locales por encima de PEAK_THRESHOLD_FRACTION de la amplitud
 * maxima del buffer, respetando una distancia minima entre picos
 * (MAX_PLAUSIBLE_F0_HZ) para no contar dos veces el mismo pulso.
 *
 * @return F0 en Hz, o 0.0f si no se detectaron al menos 2 picos validos.
 */
static float F0EstimateFromBuffer(const float *buffer, uint32_t length, float sample_rate_hz){
    if(length < 2){
        return 0.0f;
    }

    float max_abs = 0.0f;
    for(uint32_t i = 0; i < length; i++){
        float abs_val = fabsf(buffer[i]);
        if(abs_val > max_abs){
            max_abs = abs_val;
        }
    }
    if(max_abs <= 0.0f){
        return 0.0f;
    }

    float threshold = max_abs * PEAK_THRESHOLD_FRACTION;
    uint32_t min_distance_samples = (uint32_t)(sample_rate_hz / MAX_PLAUSIBLE_F0_HZ);
    if(min_distance_samples < 1){
        min_distance_samples = 1;
    }

    uint32_t last_peak_index = 0;
    bool has_last_peak = false;
    double sum_periods = 0.0;
    uint32_t period_count = 0;

    for(uint32_t i = 1; i + 1 < length; i++){
        bool is_local_max = (buffer[i] > buffer[i - 1]) && (buffer[i] >= buffer[i + 1]);
        bool above_threshold = buffer[i] >= threshold;

        if(is_local_max && above_threshold){
            if(!has_last_peak){
                last_peak_index = i;
                has_last_peak = true;
            } else if((i - last_peak_index) >= min_distance_samples){
                sum_periods += (double)(i - last_peak_index);
                period_count++;
                last_peak_index = i;
            }
            /* si esta demasiado cerca del pico anterior, se descarta */
        }
    }

    if(period_count == 0){
        return 0.0f;
    }

    double avg_period_samples = sum_periods / (double)period_count;
    return (float)(sample_rate_hz / avg_period_samples);
}

/**
 * @brief Clasifica genero segun F0 (hombre: 85-180 Hz, mujer: 165-255 Hz;
 * 165-180 Hz es ambiguo porque ambos rangos se superponen ahi).
 */
static voice_gender_t F0ClassifyGender(float f0_hz){
    bool in_male_range   = (f0_hz >= 85.0f)  && (f0_hz <= 180.0f);
    bool in_female_range = (f0_hz >= 165.0f) && (f0_hz <= 255.0f);

    if(in_male_range && in_female_range){
        return VOICE_GENDER_AMBIGUOUS;
    }
    if(in_male_range){
        return VOICE_GENDER_MALE;
    }
    if(in_female_range){
        return VOICE_GENDER_FEMALE;
    }
    return VOICE_GENDER_UNDETERMINED;
}

/*---------- Orquestacion del test guiado de voz ----------------------------*/

/**
 * @brief Callback del timer de muestreo a 1600 Hz.
 */
static void FuncTimerSample(void *param){
    vTaskNotifyGiveFromISR(sample_task_handle, pdFALSE);
}

/**
 * @brief Espera (bloqueante, por polling) a que se presione SWITCH_1
 * (siguiente) o SWITCH_2 (reintentar), con un debounce simple que espera a
 * que el switch se suelte antes de retornar.
 *
 * @return true si se presiono SWITCH_1 (siguiente); false si SWITCH_2
 * (reintentar).
 */
static bool WaitForNextOrRetry(void){
    int8_t pressed;

    do {
        pressed = SwitchesRead();
        vTaskDelay(pdMS_TO_TICKS(20));
    } while((pressed & (SWITCH_1 | SWITCH_2)) == 0);

    bool advance = (pressed & SWITCH_1) != 0;

    do {
        vTaskDelay(pdMS_TO_TICKS(20));
    } while(SwitchesRead() != 0);

    return advance;
}

/**
 * @brief Arranca (o reinicia) un paso del test guiado de voz.
 */
static void VoiceTestStartPhase(voice_test_phase_t phase){
    voice_test_phase = phase;
    voice_test_sample_count = 0;
    RmsBlockReset(&voice_test_rms_accum);

    switch(phase){
        case VOICE_TEST_PHASE_SILENCE:
            printf("\r\n=== Paso 1/3: Quedese quieto y en silencio durante %d segundos ===\r\n",
                   VOICE_TEST_SILENCE_SEC);
            break;
        case VOICE_TEST_PHASE_COMFORTABLE:
            printf("\r\n=== Paso 2/3: Diga /a/ sostenida, con voz comoda, durante %d segundos ===\r\n",
                   VOICE_TEST_COMFORTABLE_SEC);
            break;
        case VOICE_TEST_PHASE_SHOUT:
            printf("\r\n=== Paso 3/3: Diga /a/ sostenida, pero GRITANDO, durante %d segundos ===\r\n",
                   VOICE_TEST_SHOUT_SEC);
            break;
        default:
            break;
    }
}

/**
 * @brief Finaliza el paso actual del test guiado: calcula el resultado,
 * lo imprime, pide confirmacion por switch y avanza o repite.
 */
static void VoiceTestFinishPhase(void){
    float level = RmsBlockGet(&voice_test_rms_accum);

    switch(voice_test_phase){
        case VOICE_TEST_PHASE_SILENCE:
            printf("Nivel de silencio (RMS): %.2f mg\r\n", level);
            break;

        case VOICE_TEST_PHASE_COMFORTABLE:
            detected_f0_hz = F0EstimateFromBuffer(f0_buffer, VOICE_TEST_COMFORTABLE_SAMPLES,
                                                   (float)SAMPLE_FREQ_HZ);
            detected_gender = F0ClassifyGender(detected_f0_hz);
            printf("Nivel de voz normal (RMS): %.2f mg\r\n", level);
            printf("F0 estimada: %.1f Hz -> ", detected_f0_hz);
            switch(detected_gender){
                case VOICE_GENDER_MALE:      printf("voz masculina (85-180 Hz)\r\n"); break;
                case VOICE_GENDER_FEMALE:    printf("voz femenina (165-255 Hz)\r\n"); break;
                case VOICE_GENDER_AMBIGUOUS: printf("rango ambiguo (165-180 Hz, se superponen)\r\n"); break;
                default:                     printf("fuera de los rangos esperados\r\n"); break;
            }
            break;

        case VOICE_TEST_PHASE_SHOUT:
            printf("Nivel de voz gritando (RMS): %.2f mg\r\n", level);
            break;

        default:
            break;
    }

    printf("Presione SWITCH_1 para continuar, o SWITCH_2 para repetir este paso.\r\n");
    bool advance = WaitForNextOrRetry();

    if(!advance){
        VoiceTestStartPhase(voice_test_phase); /* reintentar: repetir el mismo paso */
        return;
    }

    switch(voice_test_phase){
        case VOICE_TEST_PHASE_SILENCE:
            level_silence_mg = level;
            VoiceTestStartPhase(VOICE_TEST_PHASE_COMFORTABLE);
            break;

        case VOICE_TEST_PHASE_COMFORTABLE:
            level_normal_mg = level;
            VoiceTestStartPhase(VOICE_TEST_PHASE_SHOUT);
            break;

        case VOICE_TEST_PHASE_SHOUT:
            level_loud_mg = level;
            VoiceLevelClassifierCalibrate(&voice_classifier, level_silence_mg, level_normal_mg, level_loud_mg);
            RmsRunningInit(&live_rms_meter, (float)SAMPLE_FREQ_HZ, LIVE_RMS_TIME_CONSTANT_SEC);
            voice_test_phase = VOICE_TEST_PHASE_LIVE;
            printf("\r\n=== Calibracion completa. Monitoreo en vivo iniciado. ===\r\n");
            printf("LED_1 (verde) = bajito, LED_2 (amarillo) = normal, LED_3 (rojo) = gritando\r\n");
            break;

        default:
            break;
    }
}

/**
 * @brief Procesa una muestra dentro del test guiado de voz: durante los
 * pasos de calibracion, acumula RMS (y bufferea la señal en el paso de voz
 * comoda, para la estimacion de F0); en modo de monitoreo en vivo,
 * clasifica el nivel actual y actualiza los LEDs si cambio de categoria.
 */
static void VoiceTestProcessSample(float filtered_value){
    switch(voice_test_phase){
        case VOICE_TEST_PHASE_SILENCE:
            RmsBlockAddSample(&voice_test_rms_accum, filtered_value);
            voice_test_sample_count++;
            if(voice_test_sample_count >= VOICE_TEST_SILENCE_SAMPLES){
                VoiceTestFinishPhase();
            }
            break;

        case VOICE_TEST_PHASE_COMFORTABLE:
            RmsBlockAddSample(&voice_test_rms_accum, filtered_value);
            if(voice_test_sample_count < VOICE_TEST_COMFORTABLE_SAMPLES){
                f0_buffer[voice_test_sample_count] = filtered_value;
            }
            voice_test_sample_count++;
            if(voice_test_sample_count >= VOICE_TEST_COMFORTABLE_SAMPLES){
                VoiceTestFinishPhase();
            }
            break;

        case VOICE_TEST_PHASE_SHOUT:
            RmsBlockAddSample(&voice_test_rms_accum, filtered_value);
            voice_test_sample_count++;
            if(voice_test_sample_count >= VOICE_TEST_SHOUT_SAMPLES){
                VoiceTestFinishPhase();
            }
            break;

        case VOICE_TEST_PHASE_LIVE: {
            float level = RmsRunningUpdate(&live_rms_meter, filtered_value);
            voice_level_t current_level = VoiceLevelClassify(&voice_classifier, level);

            if(current_level != last_voice_level){
                uint8_t mask;
                switch(current_level){
                    case VOICE_LEVEL_SOFT:   mask = LED_1; break; /* verde */
                    case VOICE_LEVEL_NORMAL: mask = LED_2; break; /* amarillo */
                    case VOICE_LEVEL_LOUD:   mask = LED_3; break; /* rojo */
                    case VOICE_LEVEL_SILENCE:
                    default:                 mask = 0;     break; /* apagados */
                }
                LedsMask(mask);
                last_voice_level = current_level;
            }
            break;
        }
    }
}

/**
 * @brief Acumula una muestra durante la ventana de calibracion de offset
 * de gravedad (10 s). Al completarse, calcula el offset de cada eje y
 * arranca el test guiado de voz.
 */
static void CalibrationUpdate(const accel_data_t *sample){
    calib_sum_x += sample->x;
    calib_sum_y += sample->y;
    calib_sum_z += sample->z;
    calib_count++;

    if(calib_count == 1){
        printf("Iniciando calibracion de offset (10 segundos). Mantenga el sensor en reposo...\r\n");
    }

    if(calib_count >= CALIBRATION_SAMPLES){
        offset_x = (float)(calib_sum_x / CALIBRATION_SAMPLES);
        offset_y = (float)(calib_sum_y / CALIBRATION_SAMPLES);
        offset_z = (float)(calib_sum_z / CALIBRATION_SAMPLES);
        is_calibrated = true;
        printf("Calibracion finalizada. Offsets -> X: %.2f mg, Y: %.2f mg, Z: %.2f mg\r\n",
               offset_x, offset_y, offset_z);

        VoiceTestStartPhase(VOICE_TEST_PHASE_SILENCE); /* arranca el test guiado de voz */
    }
}

/**
 * @brief Resta el offset de continua y aplica el filtro IIR (HPF 1 Hz ->
 * LPF 600 Hz) sobre el eje activo configurado. Deja el resultado en
 * filtered_out (para poder transmitirlo) y devuelve el valor del eje
 * activo (el que usa el test de voz).
 */
static float ComputeFilteredAxisValue(accel_data_t *sample, accel_data_t *filtered_out){
    sample->x -= offset_x;
    sample->y -= offset_y;
    sample->z -= offset_z;

    switch(SELECTED_AXIS){
        case AXIS_X:
            HiPassFilter(&sample->x, &filtered_out->x, 1);
            LowPassFilter(&filtered_out->x, &filtered_out->x, 1);
            return filtered_out->x;

        case AXIS_Y:
            HiPassFilter(&sample->y, &filtered_out->y, 1);
            LowPassFilter(&filtered_out->y, &filtered_out->y, 1);
            return filtered_out->y;

        case AXIS_Z:
            HiPassFilter(&sample->z, &filtered_out->z, 1);
            LowPassFilter(&filtered_out->z, &filtered_out->z, 1);
            return filtered_out->z;

        case AXIS_ALL:
        default:
            /* Nota: HiPassFilter/LowPassFilter no reciben un identificador
             * de canal, por lo que todo indica que mantienen un unico
             * estado interno compartido. Aplicarlos secuencialmente a x, y,
             * z mezcla la historia de los 3 ejes entre si (limitacion
             * preexistente de iir_filter con AXIS_ALL, no introducida por
             * este cambio). El test de voz, en este caso, usa el eje Y
             * como referencia. */
            HiPassFilter(&sample->x, &filtered_out->x, 1);
            LowPassFilter(&filtered_out->x, &filtered_out->x, 1);
            HiPassFilter(&sample->y, &filtered_out->y, 1);
            LowPassFilter(&filtered_out->y, &filtered_out->y, 1);
            HiPassFilter(&sample->z, &filtered_out->z, 1);
            LowPassFilter(&filtered_out->z, &filtered_out->z, 1);
            return filtered_out->y;
    }
}

/**
 * @brief Formatea y transmite por UART la muestra filtrada, segun el eje
 * activo configurado.
 */
static void TransmitFilteredSample(const accel_data_t *filtered){
    char linea[64];

    switch(SELECTED_AXIS){
        case AXIS_X:
            snprintf(linea, sizeof(linea), "%.2f\r\n", filtered->x);
            break;
        case AXIS_Y:
            snprintf(linea, sizeof(linea), "%.2f\r\n", filtered->y);
            break;
        case AXIS_Z:
            snprintf(linea, sizeof(linea), "%.2f\r\n", filtered->z);
            break;
        case AXIS_ALL:
        default:
            snprintf(linea, sizeof(linea), "%.2f,%.2f,%.2f\r\n", filtered->x, filtered->y, filtered->z);
            break;
    }

    UartSendString(UART_PC, linea);
}

/**
 * @brief Tarea unica: lee el ADXL345, calibra el offset de gravedad
 * (primeros 10 s), y a partir de ahi filtra, transmite por UART y alimenta
 * el test guiado de voz / monitoreo en vivo con cada muestra.
 */
static void SampleTask(void *param){
    adxl345_accel_raw_t raw;
    accel_data_t sample;
    accel_data_t filtered;

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(adxl345_read_accel(&raw) == ADXL345_OK){
            sample.x = (float)adxl345_raw_to_mg(raw.x, true, ADXL345_RANGE_2G);
            sample.y = (float)adxl345_raw_to_mg(raw.y, true, ADXL345_RANGE_2G);
            sample.z = (float)adxl345_raw_to_mg(raw.z, true, ADXL345_RANGE_2G);

            if(!is_calibrated){
                CalibrationUpdate(&sample);
                continue; /* durante la calibracion de offset no se transmite */
            }

            float scalar_value = ComputeFilteredAxisValue(&sample, &filtered);
            TransmitFilteredSample(&filtered);
            VoiceTestProcessSample(scalar_value);
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

    /* --- Inicializacion de LEDs y switches (test guiado de voz) --- */
    LedsInit();
    LedsOffAll();
    SwitchesInit();

    /* --- Creacion de la unica tarea de muestreo --- */
    xTaskCreate(&SampleTask, "sample_adxl345", 4096, NULL, 5, &sample_task_handle);

    /* --- Configuracion y arranque de Timer --- */
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