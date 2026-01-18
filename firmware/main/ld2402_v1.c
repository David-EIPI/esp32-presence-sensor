#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_system.h"
#include "clock.h"
#include "main.h"
#include "ld2402.h"

/* Shared variables */

uint8_t ld2402_calibrationStatus = 0;
int32_t ld2402_calibrationNoiseGates = 0;

int32_t ld2402_MaxDistance = 200;
uint8_t ld2402_Interference = 0;
uint8_t ld2402_MotionDetected = 0;
uint8_t ld2402_StillDetected = 0; // micro-motion
int32_t ld2402_Distance = 0; // detection distance, cm
int32_t ld2402_Timeout = 1;

int32_t ld2402_trigger_threshold = 30;
int32_t ld2402_keep_threshold    = 20;
int32_t ld2402_micro_threshold   = 30;

/* Calibration progress */
int32_t ld2402_calibrationProgress = 100;


/* Local constants and variables */
static const int RX_BUF_SIZE = 512;
static const int TX_BUF_SIZE = 256;

#define MAX_TX_BUF_WORDS 32

static const char *RX_TASK_TAG = "RX_TASK";

#define TXD_PIN (GPIO_NUM_0)
#define RXD_PIN (GPIO_NUM_1)

#define LED_PIN (GPIO_NUM_13)


/* Timer and time watch data */

static gptimer_handle_t timer = 0;
static const uint32_t timer_resolution = 10 * 1000;

#define clock(...) gpclock(timer)

/* -1 = unknown, otherwise version, e.g. 303050 = 3.3.5 */
static int ld2402_fw_version = -1;

/* -1 = unknown, otherwise = status (0 = normal, 1 = error) */
//static int ld2402_poweronGainStatus = -1;

static uint32_t ack_wait_timeout = 10;

/* Communication state flag: 0 = idle, 1 = waiting for response  */
enum {
    COMM_STATE_READY,
    COMM_STATE_WAIT_ACK
};
static char ld2402_comm_state = COMM_STATE_READY;

/**********
*  UART interface functions
**********/

static void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


static size_t uart_get_chars(char *buffer, int maxsize)
{
    size_t size = 0;
    uart_get_buffered_data_len(UART_NUM_1, &size);

    if (size > maxsize)
        size = maxsize;

    if (size > 0) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, buffer, size, 100 / portTICK_PERIOD_MS);
        if (rxBytes < 0)
            size = 0;
        else if ((size_t)rxBytes < size)
            size = (size_t)rxBytes;
    } else {
/* Pause to allow some data arrive to the buffer */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return size;
}


/**********
*  GPIO for power supply
**********/

static void init_gpio(void)
{
//    int pins[] = {PWR1_PIN, PWR2_PIN, PWR3_PIN};

    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1 << LED_PIN), // | (1ULL << PWR1_PIN) | (1ULL << PWR2_PIN) | (1ULL << PWR3_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);
/*
    int i;
    for (i = 0; i < lengthof(pins); i++) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_drive_capability(pins[i], GPIO_DRIVE_CAP_3));
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(pins[i], POWER_PIN_ON_LEVEL));
    }

    for (i = 0; i < lengthof(pins); i++) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(pins[i], GPIO_MODE_OUTPUT));
    }

*/
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));
}






static void send_command(uint16_t command, uint8_t *data_bytes, unsigned count);

static void handle_off_message(void);
static void handle_distance_message(uint16_t distance_value);
static void handle_command_frame(const uint8_t *frame, uint16_t body_len);
static void handle_data_frame(const uint8_t *frame, uint16_t body_len);

/* Tunables */
#define RX_CHUNK_SIZE        64u
#define FRAME_BUF_SIZE      256u   /* must hold prefix+len+body+suffix */
#define MAX_CMD_BODY_LEN    240u   /* <= FRAME_BUF_SIZE - 10 */

/* Rolling pack (big-endian style) for stable token constants */
static inline uint32_t pack4_be(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return ((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | (uint32_t)d;
}

enum {
    CMD_PREFIX32 = 0xFDFCFBFAu,
    CMD_SUFFIX32 = 0x04030201u,
    DATA_PREFIX32 = 0xF4F3F2F1u,
    DATA_SUFFIX32 = 0xF8F7F6F5u,
};

static inline uint16_t u16_le(uint8_t lo, uint8_t hi) { return (uint16_t)lo | ((uint16_t)hi << 8); }

/* Accept both CRLF and LFCR; track last char to confirm pair */
static inline int is_term_pair(uint8_t prev, uint8_t cur)
{
    return (prev == '\r' && cur == '\n') || (prev == '\n' && cur == '\r');
}

/* Parser states */
typedef enum {
    ST_IDLE = 0,

    /* ASCII OFF parsing */
    ST_OFF_O,
    ST_OFF_OF,
    ST_OFF_OFF,
    ST_OFF_TERM1,

    /* ASCII distance parsing */
    ST_DIST_0,  /* matching "distance:" */
    ST_DIST_NUM, /* parsing digits */
    ST_DIST_TERM1,

    /* Binary command parsing */
    ST_BIN_LEN0,
    ST_BIN_LEN1,
    ST_BIN_BODY,
    ST_BIN_SUFFIX /* after body, we are watching for suffix */
} parser_state_t;

void process_serial(void)
{
    static char rx[RX_CHUNK_SIZE];

    /* Rolling last-4-bytes window */
    static uint32_t w4 = 0;
    static unsigned w4_fill = 0;

    /* Common previous byte for terminator pair detection */
    static uint8_t prev = 0;

    /* ASCII match helpers */
    static parser_state_t st = ST_IDLE;

    /* distance: prefix matcher */
    static const uint8_t DIST_PREFIX[] = "distance:";
    static size_t dist_idx = 0;
    static uint32_t dist_acc = 0; /* accumulate decimal */
    static int dist_any = 0;

    /* Binary frame buffer (contiguous frame for handler) */
    static uint8_t frame[FRAME_BUF_SIZE];
    static size_t frame_len = 0;
    static uint16_t body_len = 0;
    static uint16_t body_read = 0;

//    for (;;)

    do {
        int r = uart_get_chars(rx, (int)sizeof(rx));
        if (r < 0) break;
        if (r == 0) continue;

        for (int i = 0; i < r; ++i) {
            uint8_t b = (uint8_t)rx[i];

            /* Update rolling 4-byte window */
            if (w4_fill < 4) {
                w4 = (w4 << 8) | b;
                w4_fill++;
            } else {
                w4 = (w4 << 8) | b;
            }

            /* -------- Highest priority: binary frame once started -------- */
            if (st == ST_BIN_LEN0 || st == ST_BIN_LEN1 || st == ST_BIN_BODY || st == ST_BIN_SUFFIX) {
                /* Continue filling frame buffer; enforce bounds */
                if (frame_len < sizeof(frame)) {
                    frame[frame_len++] = b;
                } else {
                    /* Overflow: reset parser; drop frame */
                    st = ST_IDLE;
                    frame_len = 0;
                    body_len = 0;
                    body_read = 0;
                    dist_idx = 0;
                    dist_acc = 0;
                    dist_any = 0;
                    prev = b;
                    continue;
                }

                if (st == ST_BIN_LEN0) {
                    body_len = b;          /* low byte */
                    st = ST_BIN_LEN1;
                } else if (st == ST_BIN_LEN1) {
                    body_len |= (uint16_t)b << 8; /* high byte */

                    if (body_len > MAX_CMD_BODY_LEN || (size_t)(body_len + 10u) > sizeof(frame)) {
                        /* Invalid length; reset */
                        st = ST_IDLE;
                        frame_len = 0;
                        body_len = 0;
                        body_read = 0;
                    } else {
                        body_read = 0;
                        st = (body_len == 0) ? ST_BIN_SUFFIX : ST_BIN_BODY;
                    }
                } else if (st == ST_BIN_BODY) {
                    body_read++;
                    if (body_read >= body_len) {
                        st = ST_BIN_SUFFIX;
                    }
                } else { /* ST_BIN_SUFFIX */
                    /* Once we have at least 4 suffix bytes, rolling window can detect it */
                    if (w4_fill >= 4 && (w4 == (uint32_t)CMD_SUFFIX32 || w4 == (uint32_t)DATA_SUFFIX32) ) {
//                        ESP_LOGI(RX_TASK_TAG, "Suffix: 0x%x Len: %u", w4, body_len);
                        /* Frame complete: prefix(4)+len(2)+body+suffix(4) */
                        if (w4 == (uint32_t)CMD_SUFFIX32) {
                            handle_command_frame(frame + 2, body_len);
                        }
                        if (w4 == (uint32_t)DATA_SUFFIX32) {
                            handle_data_frame(frame + 2, body_len);
                        }
                        /* Reset for next message */
                        st = ST_IDLE;
                        frame_len = 0;
                        body_len = 0;
                        body_read = 0;

                        /* Also reset ASCII trackers to avoid cross-contamination */
                        dist_idx = 0;
                        dist_acc = 0;
                        dist_any = 0;
                    }
                }

                prev = b;
                continue;
            }

            /* -------- Not in binary: detect binary prefix quickly -------- */
            if (w4_fill >= 4 && (w4 == (uint32_t)CMD_PREFIX32 || w4 == (uint32_t)DATA_PREFIX32)) {
//                ESP_LOGI(RX_TASK_TAG, "CMD_PREFIX32 received");
                /* Start a new binary frame; frame begins with the 4-byte prefix.
                 */
                frame_len = 0;

                st = ST_BIN_LEN0;
                body_len = 0;
                body_read = 0;

                /* Reset ASCII matchers */
                dist_idx = 0;
                dist_acc = 0;
                dist_any = 0;

                prev = b;
                continue;
            }

            /* -------- ASCII parsing (only when not in binary) -------- */
            switch (st) {
                /* OFF\n\r or OFF\r\n */
                case ST_IDLE:
                    /* Fast path: only two ASCII message families exist. */
                    if (b == 'O') {
                        st = ST_OFF_O;
                    } else if (b == 'd') {
                        st = ST_DIST_0;
                        dist_idx = 1; /* matched 'd' */
                    } else {
                        st = ST_IDLE;
                    }
                    break;

                case ST_OFF_O:
                    st = (b == 'F') ? ST_OFF_OF : ST_IDLE;
                    break;

                case ST_OFF_OF:
                    st = (b == 'F') ? ST_OFF_OFF : ST_IDLE;
                    break;

                case ST_OFF_OFF:
                    /* Next should be first terminator char (\r or \n) */
                    if (b == '\r' || b == '\n') {
                        st = ST_OFF_TERM1;
                    } else {
                        st = ST_IDLE;
                    }
                    break;

                case ST_OFF_TERM1:
                    /* Accept terminator pair */
                    if (is_term_pair(prev, b)) {
                        handle_off_message();
                    }
                    st = ST_IDLE;
                    break;

                /* distance: ###\r\n (or \n\r) */
                case ST_DIST_0:
                    if (dist_idx < (sizeof(DIST_PREFIX) - 1)) {
                        if (b == DIST_PREFIX[dist_idx]) {
                            dist_idx++;
                            if (dist_idx == (sizeof(DIST_PREFIX) - 1)) {
                                /* Matched "distance:" fully */
                                st = ST_DIST_NUM;
                                dist_acc = 0;
                                dist_any = 0;
                            } else {
                                st = ST_DIST_0;
                            }
                        } else {
                            /* If this byte could be a new start ('d'), restart; else idle */
                            if (b == 'd') {
                                st = ST_DIST_0;
                                dist_idx = 1;
                            } else if (b == 'O') {
                                st = ST_OFF_O;
                            } else {
                                st = ST_IDLE;
                            }
                        }
                    } else {
                        st = ST_IDLE;
                    }
                    break;

                case ST_DIST_NUM:
                    if (b == ' ' || b == '\t') {
                        /* allow spaces after colon */
                    } else if (b >= '0' && b <= '9') {
                        dist_any = 1;
                        dist_acc = dist_acc * 10u + (uint32_t)(b - '0');
                        if (dist_acc > 65535u) {
                            /* overflow -> drop line */
                            st = ST_IDLE;
                        }
                    } else if (b == '\r' || b == '\n') {
                        st = ST_DIST_TERM1;
                    } else {
                        /* unexpected char -> drop line */
                        st = ST_IDLE;
                    }
                    break;

                case ST_DIST_TERM1:
                    if (is_term_pair(prev, b)) {
                        if (dist_any) {
                            handle_distance_message((uint16_t)dist_acc);
                        }
                    }
                    st = ST_IDLE;
                    break;

                default:
                    st = ST_IDLE;
                    break;
            }

            prev = b;
        }
    } while (0);
}

#include "ld2402_defs.h"
enum {
    CFG_IDLE,
    CFG_BEGIN,
    CFG_PARAM_MAXDIST,
    CFG_PARAM_TIMEOUT,
    CFG_PARAM_INTERFERENCE,
    CFG_DATA_MODE,
    CFG_END,
};

enum {
    CAL_IDLE,
    CAL_BEGIN,
    CAL_AUTO_GAIN,
    CAL_AUTO_THRESHOLD,
    CAL_AUTO_PROGRESS,
    CAL_SAVE,
    CAL_END,
};

static uint8_t configStep = CFG_IDLE;
static uint8_t nextConfigStep = CFG_IDLE;

static uint8_t calibrationStep = CAL_IDLE;
static uint8_t nextCalibrationStep = CAL_IDLE;

static void handle_config_param(const uint8_t *frame, size_t body_len)
{
    if (body_len >= 4) {
        uint32_t param_value = frame[0];// pack4_be(frame[3], frame[2], frame[1], frame[0]);
        switch (configStep) {
            case CFG_PARAM_INTERFERENCE:
                ESP_LOGI(RX_TASK_TAG, "PS Interference: %lu", param_value);
                ld2402_Interference = param_value;
                break;
            default:
                break;
        }

        uint16_t command_data[3];
        switch (calibrationStep) {
            case CAL_SAVE:
                param_value = pack4_be(frame[3], frame[2], frame[1], frame[0]);
                ESP_LOGI(RX_TASK_TAG, "Gate 0x3F: %lu", param_value);
                command_data[0] = PARAM_SAVE_TRIGGER;
                command_data[1] = param_value & (uint16_t)-1;
                command_data[2] = param_value >> 16;
                send_command(CMD_SET_PARAMS, (uint8_t *)&command_data, sizeof(command_data));
                break;
            default:
                break;
        }

    }
}


static void handle_off_message(void)
{
//    ESP_LOGI(RX_TASK_TAG, "OFF");
    ld2402_MotionDetected = 0;
    ld2402_StillDetected = 0;
    ld2402_Distance = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, ld2402_MotionDetected));
}

static void handle_distance_message(uint16_t distance_value)
{
//    ESP_LOGI(RX_TASK_TAG, "Distance: %u", distance_value);
    ld2402_Distance = distance_value;
    ld2402_MotionDetected = 1;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, ld2402_MotionDetected));
}

static void handle_version_frame(const uint8_t *frame, size_t frame_len)
{
    if (frame_len > 2) {
        ld2402_fw_version = version_to_u32(frame+2, frame[0]);
        ESP_LOGI(RX_TASK_TAG, "FW Ver: 0x%06x", ld2402_fw_version);
    }
    light_driver_set_power(0, 0, 0);
}

static void handle_command_frame(const uint8_t *frame, uint16_t body_len)
{
    ld2402_comm_state = COMM_STATE_READY;

    if (body_len < 4)
        return;

    uint16_t cmd = frame[0] + (frame[1] << 8);
    uint8_t ack = frame[2] + (frame[3] << 8);
    ESP_LOGI(RX_TASK_TAG, "CMD 0x%x (%u bytes): ack=%u", cmd, body_len, ack);

    if (ack != 0 && ((cmd & 0xff) != CMD_CALIBRATION_INTERFERENCE))
        return;

    frame += 4;
    body_len -= 4;
/* All response messages have 8th bit set, flipping this bit will produce the original command */
    switch (cmd ^ 0x100) {
        case CMD_GET_VERSION:
            handle_version_frame(frame, body_len);
            break;
        case CMD_GET_PARAMS:
            handle_config_param(frame, body_len);
            configStep = nextConfigStep;
            calibrationStep = nextCalibrationStep;
            break;
            /* Configuration mode successfully enabled or disabled */
        case CMD_ENABLE_CONFIG:
            ESP_LOGI(RX_TASK_TAG, "Proto ver: %d bufsize: %d", frame[0] + (frame[1] << 8), frame[2] + (frame[3] << 8));
             __attribute__((__fallthrough__)); /* fall through */
        case CMD_SET_MODE:
        case CMD_DISABLE_CONFIG:
        case CMD_SET_PARAMS:
            /* Parameter was successfully set, continue to the next step. */
            /* Currently this is one of the PARAM_MAX_DISTANCE, PARAM_TIMEOUT, PARAM_SAVE_TRIGGER */
            configStep = nextConfigStep;
            calibrationStep = nextCalibrationStep;
            break;

        case CMD_START_CALIBRATION:
            calibrationStep = nextCalibrationStep;
            break;

        case CMD_GET_CALIBRATION_STATUS:
            ld2402_calibrationProgress = u16_le(frame[0], frame[1]);
            ESP_LOGI(RX_TASK_TAG, "Calibration: %u", ld2402_calibrationProgress);
            if (ld2402_calibrationProgress >= 100)
                calibrationStep = nextCalibrationStep;
            break;
        case CMD_CALIBRATION_INTERFERENCE:
            ld2402_calibrationStatus = ack;
            ld2402_calibrationNoiseGates = u16_le(frame[0], frame[1]);
            calibrationStep = nextCalibrationStep;
            ESP_LOGI(RX_TASK_TAG, "Calibration: %u %x", ld2402_calibrationStatus, ld2402_calibrationNoiseGates);
            break;
        case CMD_AUTO_GAIN:
            calibrationStep = nextCalibrationStep;
            break;
        case CMD_AUTO_GAIN_COMPLETE:
            ESP_LOGI(RX_TASK_TAG, "Auto gain completed");
            break;
        default:
            break;
    }
}

static void handle_data_frame(const uint8_t *frame, uint16_t body_len)
{
    ld2402_MotionDetected = frame[0] == 1;
    ld2402_StillDetected = frame[0] == 2;
    ld2402_Distance = u16_le(frame[1], frame[2]);
//    ESP_LOGI(RX_TASK_TAG, "DATA (%u bytes): %u %u", body_len, ld2402_MotionDetected, ld2402_Distance);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, ld2402_MotionDetected));
}

static void send_command(uint16_t command, uint8_t *data_bytes, unsigned count)
{
    uint32_t frame_header = 0xFAFBFCFD;
    uint32_t frame_footer = 0x01020304;

    uint16_t frame_len = count + 2;

    uart_write_bytes(UART_NUM_1, &frame_header, sizeof(frame_header));
    uart_write_bytes(UART_NUM_1, &frame_len, sizeof(frame_len));
    uart_write_bytes(UART_NUM_1, &command, sizeof(command));

    if (count)
        uart_write_bytes(UART_NUM_1, data_bytes, count);

    uart_write_bytes(UART_NUM_1, &frame_footer, sizeof(frame_footer));

    ld2402_comm_state = COMM_STATE_WAIT_ACK;
    ESP_LOGI(RX_TASK_TAG, "Send 0x%x (%lu bytes)", command, frame_len);
}


static void update_configuration(void)
{
    uint16_t set_parameter_data[3];

    switch(configStep) {
        case CFG_BEGIN:
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;

    /* Update the parameter */
        case CFG_PARAM_MAXDIST:
            set_parameter_data[0] = PARAM_MAX_DISTANCE;
            set_parameter_data[1] = ld2402_MaxDistance & (uint16_t)-1;
            set_parameter_data[2] = ld2402_MaxDistance >> 16;
            send_command(CMD_SET_PARAMS, (uint8_t *)&set_parameter_data, sizeof(set_parameter_data));
            break;

    /* Update the parameter */
        case CFG_PARAM_TIMEOUT:
            set_parameter_data[0] = PARAM_TIMEOUT;
            set_parameter_data[1] = ld2402_Timeout & (uint16_t)-1;
            set_parameter_data[2] = ld2402_Timeout >> 16;
            send_command(CMD_SET_PARAMS, (uint8_t *)&set_parameter_data, sizeof(set_parameter_data));
            break;
    /* Read the parameter */
        case CFG_PARAM_INTERFERENCE:
            set_parameter_data[0] = PARAM_POWER_INTERFERENCE;
            send_command(CMD_GET_PARAMS, (uint8_t *)&set_parameter_data, sizeof(set_parameter_data[0]));
            break;

    /* Set engineering output mode for more details */
        case CFG_DATA_MODE:
            set_parameter_data[0] = 0;
            set_parameter_data[1] = MODE_ENGINEERING & 0xffff;
            set_parameter_data[2] = MODE_ENGINEERING >> 16;
            send_command(CMD_SET_MODE, (uint8_t *)&set_parameter_data, sizeof(set_parameter_data));
            break;

        case CFG_END:
            send_command(CMD_DISABLE_CONFIG, NULL, 0);
            break;
        default:
            return;
    }

    nextConfigStep = configStep + 1;
    if (nextConfigStep > CFG_END)
        nextConfigStep = CFG_IDLE;
}


static void do_calibration(void)
{
    uint16_t command_data[3];

    switch (calibrationStep) {
        case CAL_BEGIN:
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;
        case CAL_AUTO_THRESHOLD:
            command_data[0] = ld2402_trigger_threshold;
            command_data[1] = ld2402_keep_threshold;
            command_data[2] = ld2402_micro_threshold;
            send_command(CMD_START_CALIBRATION, (uint8_t *)&command_data, sizeof(command_data));
            break;
        case CAL_AUTO_PROGRESS:
            vTaskDelay(pdMS_TO_TICKS(1000));
            send_command(CMD_GET_CALIBRATION_STATUS, NULL, 0);
            break;
        case CAL_AUTO_GAIN:
            send_command(CMD_CALIBRATION_INTERFERENCE, NULL, 0);
            break;
        case CAL_SAVE:
            command_data[0] = 0x3F;
            send_command(CMD_GET_PARAMS, (uint8_t *)&command_data, sizeof(command_data[0]));
            break;
        case CAL_END:
            send_command(CMD_DISABLE_CONFIG, NULL, 0);
            break;
        default:
            break;
    }

    nextCalibrationStep = calibrationStep + 1;
    if (nextCalibrationStep > CAL_END)
        nextCalibrationStep = CAL_IDLE;
}

void ld2402_task(void *arg)
{
/* Indicate red color until the sensor is detected */
    light_driver_init(255, 0, 0);
    init_gpio();
    init_uart();

/* This delay is needed for UART to become active */
    vTaskDelay(pdMS_TO_TICKS(2000));

    init_timer(&timer, timer_resolution);

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint64_t clock_now = 0, clock_prev = 0;
    uint64_t ack_timer = 0;
    uint64_t cfg_timer = UINT_MAX;

    while (1) {
        clock_now = clock();
        uint64_t delta = clock_now - clock_prev;
        clock_prev = clock_now;

        cfg_timer += delta;
        ack_timer += delta;

    /* Perform init/config/monitoring tasks */
        if (COMM_STATE_READY == ld2402_comm_state) {
            ack_timer = 0;

            if (ld2402_fw_version < 0) {
                send_command(CMD_GET_VERSION, NULL, 0);
                continue;
            }

            if (CFG_IDLE == configStep && CAL_IDLE == calibrationStep) {
                if (cfg_timer > timer_resolution * 10000) {
//                    ESP_LOGI(RX_TASK_TAG, "Config start");
                    configStep = CFG_BEGIN;
                    cfg_timer = 0;
                } else {
                    if (0 == ld2402_calibrationProgress) {
                        calibrationStep = CAL_BEGIN;
                    }
                }
            }

            if (CFG_IDLE != configStep) {
                update_configuration();
                if (cfg_timer > timer_resolution * 30) {
                    /* Configuration takes too long */
                    configStep = CFG_IDLE;
                    cfg_timer = 0;
                }
            }

            if (CAL_IDLE != calibrationStep) {
                do_calibration();
            }
        } else {
    /* Ack not received within timeout period - exit configuration mode */
            if (ack_timer > ack_wait_timeout * timer_resolution) {
                ld2402_comm_state = COMM_STATE_READY;
                if (CFG_IDLE != configStep)
                    configStep = CAL_END;
                if (CAL_IDLE != calibrationStep)
                    calibrationStep = CAL_END;
            }
        }

        process_serial();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

