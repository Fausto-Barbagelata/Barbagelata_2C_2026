/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "adxl345.h"
#include "iir_filter.h"
#include "led.h"
#include "switch.h"
#include "ble_mcu.h"

/*==================[macros and definitions]=================================*/
#define UART_BAUD_RATE          921600
#define SAMPLE_FREQ_HZ          1600
#define SAMPLE_PERIOD_US        (1000000 / SAMPLE_FREQ_HZ)

/** @brief Factor de submuestreo para transmisión BLE
 *  1600 Hz / 16 = 100 Hz enviados por Bluetooth (ideal para la app) */
#define BLE_DOWNSAMPLE_FACTOR   16

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

/** @brief Eje activo seleccionado para la visualizacion */
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

#define SILENCE_MARGIN_FRACTION   0.2f
#define PEAK_THRESHOLD_FRACTION   0.5f
#define MAX_PLAUSIBLE_F0_HZ       350.0f

#define LED_TASK_PERIOD_MS        50
#define SWITCH_TASK_PERIOD_MS     20

typedef enum {
    VOICE_TEST_PHASE_SILENCE = 0,
    VOICE_TEST_PHASE_COMFORTABLE,
    VOICE_TEST_PHASE_SHOUT,
    VOICE_TEST_PHASE_LIVE,
} voice_test_phase_t;

typedef enum {
    VOICE_LEVEL_SILENCE = 0,
    VOICE_LEVEL_SOFT,
    VOICE_LEVEL_NORMAL,
    VOICE_LEVEL_LOUD,
} voice_level_t;

typedef enum {
    VOICE_GENDER_UNDETERMINED = 0,
    VOICE_GENDER_MALE,
    VOICE_GENDER_FEMALE,
    VOICE_GENDER_AMBIGUOUS,
} voice_gender_t;

typedef enum {
    SWITCH_EVENT_NONE = 0,
    SWITCH_EVENT_NEXT,
    SWITCH_EVENT_RETRY,
} switch_event_t;

typedef struct {
    double   sum_sq;
    uint32_t count;
} rms_block_t;

typedef struct {
    float mean_sq;
    float alpha;
} rms_running_t;

typedef struct {
    float threshold_silence_to_soft;
    float threshold_soft_to_normal;
    float threshold_normal_to_loud;
    bool  calibrated;
} voice_level_classifier_t;

/*==================[internal data definition]===============================*/

TaskHandle_t acquisition_task_handle = NULL;
TaskHandle_t processing_task_handle  = NULL;
TaskHandle_t led_task_handle         = NULL;
TaskHandle_t switch_task_handle      = NULL;

accel_data_t g_latest_sample;
voice_level_t g_display_voice_level = VOICE_LEVEL_SILENCE;
switch_event_t g_switch_event = SWITCH_EVENT_NONE;

float offset_x = 0.0f;
float offset_y = 0.0f;
float offset_z = 0.0f;
bool is_calibrated = false;

double calib_sum_x = 0.0;
double calib_sum_y = 0.0;
double calib_sum_z = 0.0;
uint32_t calib_count = 0;

voice_test_phase_t voice_test_phase = VOICE_TEST_PHASE_SILENCE;
uint32_t voice_test_sample_count = 0;
rms_block_t voice_test_rms_accum;

bool voice_test_awaiting_confirmation = false;
float voice_test_pending_level = 0.0f;

float level_silence_mg = 0.0f;
float level_normal_mg  = 0.0f;
float level_loud_mg    = 0.0f;

voice_level_classifier_t voice_classifier;

float f0_buffer[VOICE_TEST_COMFORTABLE_SAMPLES];
float detected_f0_hz = 0.0f;
voice_gender_t detected_gender = VOICE_GENDER_UNDETERMINED;

rms_running_t live_rms_meter;

/*==================[internal functions definition]==========================*/

static void RmsBlockReset(rms_block_t *meter){
    meter->sum_sq = 0.0;
    meter->count = 0;
}

static void RmsBlockAddSample(rms_block_t *meter, float sample){
    meter->sum_sq += (double)sample * (double)sample;
    meter->count++;
}

static float RmsBlockGet(const rms_block_t *meter){
    if(meter->count == 0){
        return 0.0f;
    }
    return (float)sqrt(meter->sum_sq / (double)meter->count);
}

static void RmsRunningInit(rms_running_t *meter, float sample_rate_hz, float time_constant_s){
    float dt = 1.0f / sample_rate_hz;
    meter->alpha = dt / (time_constant_s + dt);
    meter->mean_sq = 0.0f;
}

static float RmsRunningUpdate(rms_running_t *meter, float sample){
    float sq = sample * sample;
    meter->mean_sq += meter->alpha * (sq - meter->mean_sq);
    return sqrtf(meter->mean_sq);
}

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
        }
    }

    if(period_count == 0){
        return 0.0f;
    }

    double avg_period_samples = sum_periods / (double)period_count;
    return (float)(sample_rate_hz / avg_period_samples);
}

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

static void FuncTimerSample(void *param){
    vTaskNotifyGiveFromISR(acquisition_task_handle, pdFALSE);
}

static void VoiceTestStartPhase(voice_test_phase_t phase){
    voice_test_phase = phase;
    voice_test_sample_count = 0;
    voice_test_awaiting_confirmation = false;
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

static void VoiceTestFinishPhase(void){
    voice_test_pending_level = RmsBlockGet(&voice_test_rms_accum);

    switch(voice_test_phase){
        case VOICE_TEST_PHASE_SILENCE:
            printf("Nivel de silencio (RMS): %.2f mg\r\n", voice_test_pending_level);
            break;

        case VOICE_TEST_PHASE_COMFORTABLE:
            detected_f0_hz = F0EstimateFromBuffer(f0_buffer, VOICE_TEST_COMFORTABLE_SAMPLES,
                                                   (float)SAMPLE_FREQ_HZ);
            detected_gender = F0ClassifyGender(detected_f0_hz);
            printf("Nivel de voz normal (RMS): %.2f mg\r\n", voice_test_pending_level);
            printf("F0 estimada: %.1f Hz -> ", detected_f0_hz);
            switch(detected_gender){
                case VOICE_GENDER_MALE:      printf("voz masculina (85-180 Hz)\r\n"); break;
                case VOICE_GENDER_FEMALE:    printf("voz femenina (165-255 Hz)\r\n"); break;
                case VOICE_GENDER_AMBIGUOUS: printf("rango ambiguo (165-180 Hz, se superponen)\r\n"); break;
                default:                     printf("fuera de los rangos esperados\r\n"); break;
            }
            break;

        case VOICE_TEST_PHASE_SHOUT:
            printf("Nivel de voz gritando (RMS): %.2f mg\r\n", voice_test_pending_level);
            break;

        default:
            break;
    }

    printf("Presione SWITCH_1 para continuar, o SWITCH_2 para repetir este paso.\r\n");
    voice_test_awaiting_confirmation = true;
}

static void VoiceTestCheckSwitchEvent(void){
    switch_event_t evt = g_switch_event;
    if(evt == SWITCH_EVENT_NONE){
        return;
    }
    g_switch_event = SWITCH_EVENT_NONE;
    voice_test_awaiting_confirmation = false;

    if(evt == SWITCH_EVENT_RETRY){
        VoiceTestStartPhase(voice_test_phase);
        return;
    }

    switch(voice_test_phase){
        case VOICE_TEST_PHASE_SILENCE:
            level_silence_mg = voice_test_pending_level;
            VoiceTestStartPhase(VOICE_TEST_PHASE_COMFORTABLE);
            break;

        case VOICE_TEST_PHASE_COMFORTABLE:
            level_normal_mg = voice_test_pending_level;
            VoiceTestStartPhase(VOICE_TEST_PHASE_SHOUT);
            break;

        case VOICE_TEST_PHASE_SHOUT:
            level_loud_mg = voice_test_pending_level;
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
            g_display_voice_level = VoiceLevelClassify(&voice_classifier, level);
            break;
        }
    }
}

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

        VoiceTestStartPhase(VOICE_TEST_PHASE_SILENCE);
    }
}

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
 * @brief Formatea y transmite por UART y BLE para la app "Bluetooth Electronics".
 * Formato Bluetooth Electronics para gráfico: "*G<valor>*"
 */
static void TransmitFilteredSample(const accel_data_t *filtered){
    char linea_uart[64];
    char linea_ble[32];
    static uint16_t downsample_cnt = 0;

    /* Transmisión Serie (UART a 1600 Hz completa) */
    switch(SELECTED_AXIS){
        case AXIS_X:
            snprintf(linea_uart, sizeof(linea_uart), "%.2f\r\n", filtered->x);
            break;
        case AXIS_Y:
            snprintf(linea_uart, sizeof(linea_uart), "%.2f\r\n", filtered->y);
            break;
        case AXIS_Z:
            snprintf(linea_uart, sizeof(linea_uart), "%.2f\r\n", filtered->z);
            break;
        case AXIS_ALL:
        default:
            snprintf(linea_uart, sizeof(linea_uart), "%.2f,%.2f,%.2f\r\n", filtered->x, filtered->y, filtered->z);
            break;
    }
    UartSendString(UART_PC, linea_uart);

    /* Transmisión Bluetooth Electronics (Submuestreado a ~100 Hz) */
    downsample_cnt++;
    if(downsample_cnt >= BLE_DOWNSAMPLE_FACTOR){
        downsample_cnt = 0;

        if(BleStatus() == BLE_CONNECTED){
            switch(SELECTED_AXIS){
                case AXIS_X:
                    snprintf(linea_ble, sizeof(linea_ble), "*G%.2f*", filtered->x);
                    break;
                case AXIS_Y:
                    snprintf(linea_ble, sizeof(linea_ble), "*G%.2f*", filtered->y);
                    break;
                case AXIS_Z:
                    snprintf(linea_ble, sizeof(linea_ble), "*G%.2f*", filtered->z);
                    break;
                case AXIS_ALL:
                default:
                    /* Envío de 3 variables delimitadas: *G<x>|H<y>|I<z>* */
                    snprintf(linea_ble, sizeof(linea_ble), "*G%.2f|H%.2f|I%.2f*", filtered->x, filtered->y, filtered->z);
                    break;
            }
            BleSendString(linea_ble);
        }
    }
}

/**
 * @brief Callback opcional para procesar comandos recibidos por Bluetooth
 */
void ReadBluetoothData(uint8_t * data, uint8_t length){
    /* Ejemplo: si desde la app se envía un botón 'R' para reiniciar o cambiar de estado */
    if(length > 0 && data[0] == 'R'){
        g_switch_event = SWITCH_EVENT_RETRY;
    }
}

/*==================[tareas]===================================================*/

static void AcquisitionTask(void *param){
    adxl345_accel_raw_t raw;

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(adxl345_read_accel(&raw) == ADXL345_OK){
            g_latest_sample.x = (float)adxl345_raw_to_mg(raw.x, true, ADXL345_RANGE_2G);
            g_latest_sample.y = (float)adxl345_raw_to_mg(raw.y, true, ADXL345_RANGE_2G);
            g_latest_sample.z = (float)adxl345_raw_to_mg(raw.z, true, ADXL345_RANGE_2G);

            xTaskNotifyGive(processing_task_handle);
        }
    }
}

static void ProcessingTask(void *param){
    accel_data_t sample;
    accel_data_t filtered;

    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sample = g_latest_sample;

        if(!is_calibrated){
            CalibrationUpdate(&sample);
            continue;
        }

        if(voice_test_awaiting_confirmation){
            VoiceTestCheckSwitchEvent();
            continue;
        }

        float scalar_value = ComputeFilteredAxisValue(&sample, &filtered);
        TransmitFilteredSample(&filtered);
        VoiceTestProcessSample(scalar_value);
    }
}

static void LedTask(void *param){
    voice_level_t last_shown = VOICE_LEVEL_SILENCE;
    LedsOffAll();

    while(true){
        voice_level_t current = g_display_voice_level;

        if(current != last_shown){
            uint8_t mask;
            switch(current){
                case VOICE_LEVEL_SOFT:   mask = LED_1; break;
                case VOICE_LEVEL_NORMAL: mask = LED_2; break;
                case VOICE_LEVEL_LOUD:   mask = LED_3; break;
                case VOICE_LEVEL_SILENCE:
                default:                 mask = 0;     break;
            }
            LedsMask(mask);
            last_shown = current;
        }

        vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

static void SwitchTask(void *param){
    while(true){
        int8_t pressed = SwitchesRead();

        if(pressed & SWITCH_1){
            g_switch_event = SWITCH_EVENT_NEXT;
            do { vTaskDelay(pdMS_TO_TICKS(SWITCH_TASK_PERIOD_MS)); } while(SwitchesRead() != 0);
        } else if(pressed & SWITCH_2){
            g_switch_event = SWITCH_EVENT_RETRY;
            do { vTaskDelay(pdMS_TO_TICKS(SWITCH_TASK_PERIOD_MS)); } while(SwitchesRead() != 0);
        }

        vTaskDelay(pdMS_TO_TICKS(SWITCH_TASK_PERIOD_MS));
    }
}

/*==================[external functions definition]==========================*/

void app_main(void){

    /* --- Inicialización BLE --- */
    ble_config_t ble_configuration = {
        .device_name = "DOSIMETRO_VOZ",
        .func_p = ReadBluetoothData
    };
    BleInit(&ble_configuration);

    /* --- Inicialización ADXL345 --- */
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

    /* --- Inicialización de Filtros IIR (1 Hz - 600 Hz) --- */
    HiPassInit(SAMPLE_FREQ_HZ, 60, ORDER_2);
    LowPassInit(SAMPLE_FREQ_HZ, 600, ORDER_2);

    /* --- Inicialización UART --- */
    serial_config_t uart_config = {
        .port      = UART_PC,
        .baud_rate = UART_BAUD_RATE,
        .func_p    = UART_NO_INT,
        .param_p   = NULL
    };
    UartInit(&uart_config);

    /* --- Inicialización LEDs y Switches --- */
    LedsInit();
    LedsOffAll();
    SwitchesInit();

    /* --- Creación de Tareas FreeRTOS --- */
    xTaskCreate(&AcquisitionTask, "acquisition", 2048, NULL, 6, &acquisition_task_handle);
    xTaskCreate(&ProcessingTask,  "processing",  4096, NULL, 5, &processing_task_handle);
    xTaskCreate(&LedTask,         "leds",        2048, NULL, 3, &led_task_handle);
    xTaskCreate(&SwitchTask,      "switches",    2048, NULL, 3, &switch_task_handle);

    /* --- Timer de Muestreo --- */
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