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
#include "ld24xx.h"

#if defined(CONFIG_LD2410S_SENSOR)

/* Shared variables */

int32_t ld24xx_MaxDistance = 200;
uint8_t ld24xx_Interference = 0;
uint8_t ld24xx_MotionDetected = 0;
uint8_t ld24xx_StillDetected = 0; // micro-motion
int32_t ld24xx_Distance = 0; // detection distance, cm
int32_t ld24xx_Timeout = 1;

int32_t ld24xx_trigger_threshold = 30;
int32_t ld24xx_keep_threshold    = 20;
int32_t ld24xx_micro_threshold   = 30;

/* Diagnostic output */
int32_t ld24xx_gate0 = 0;
int32_t ld24xx_gate3 = 0;

/* Calibration timer */
int32_t ld24xx_calibrationTimer = 0;
static int32_t req_calibration_time = 0;

/* Factory reset requested via Zigbee */
volatile int32_t resetRequest = 0;

/* Local constants and variables */
static const int RX_BUF_SIZE = 512;
static const int TX_BUF_SIZE = 256;

#define MAX_TX_BUF_WORDS 32

static const char *RX_TASK_TAG = "RX_TASK";

#define TXD_PIN (GPIO_NUM_0)
#define RXD_PIN (GPIO_NUM_1)

//#define TXD_PIN (GPIO_NUM_10)
//#define RXD_PIN (GPIO_NUM_25)

#define LED_PIN (GPIO_NUM_13)


/* Timer and time watch data */

static gptimer_handle_t timer = 0;
static const uint32_t timer_resolution = 10 * 1000;

#define clock(...) gpclock(timer)

/* -1 = unknown, otherwise version, e.g. 303050 = 3.3.5 */
static int ld24xx_fw_version = -1;

static uint32_t ack_wait_timeout = 10;

/* Communication state flag: 0 = idle, 1 = waiting for response  */
enum {
    COMM_STATE_READY,
    COMM_STATE_WAIT_ACK
};
static char ld24xx_comm_state = COMM_STATE_READY;

/* Engineering data mode flag */
uint8_t ld24xx_eng_mode = 0;

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


static void init_gpio(void)
{
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1 << LED_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));
}


static void send_command(uint16_t command, uint8_t *data_bytes, unsigned count);

/*
 * HiLink LD2410S serial stream parser (strict footers, optimized for fixed formats)
 *
 * Strict rule: For extended frames and command frames, the 4-byte footer MUST appear
 * immediately after the payload. If not, discard the entire frame as invalid.
 *
 * Frame types handled:
 * 1) Minimal: 0x6E, state, distLE(2), 0x62
 * 2) Extended: hdr F4 F3 F2 F1, lenLE(2), type 0x01 or 0x03, payload, ftr F8 F7 F6 F5
 *    - type 0x01 payload includes state(1), distLE(2), energy(64) [ignored]
 *    - type 0x03 payload includes progressLE(2)
 * 3) Command: hdr FD FC FB FA, lenLE(2), cmdLE(2), codeLE(2), data..., ftr 04 03 02 01
 *
 * Notes:
 * - Uses rolling 4-byte window for header detection only.
 * - Reads footers as the next 4 bytes (strict) rather than scanning for them.
 * - Avoids memmove/ring buffer; streaming DFA.
 * - Only standard headers: stdint.h, stddef.h
 */

int read_serial(char *buffer, int maxsize);

/* ---------- Handlers (you implement) ---------- */
static void handle_minimal_detection(uint8_t state, uint16_t distance);
static void handle_extended_detection(uint8_t state, uint16_t distance);
static void handle_calibration_progress(uint16_t progress);
static void handle_command_frame(uint16_t cmd_word,
                                 uint16_t ack_code,
                                 const uint8_t *data,
                                 size_t data_len);

/* ---------- Tunables ---------- */
#define RX_CHUNK_SIZE           128u

/* Buffers hold payload only (excluding header, length bytes, footer) */
#define EXT_PAYLOAD_BUF_SIZE   192u
#define CMD_PAYLOAD_BUF_SIZE   256u

#define MAX_EXT_PAYLOAD_LEN    (EXT_PAYLOAD_BUF_SIZE)
#define MAX_CMD_PAYLOAD_LEN    (CMD_PAYLOAD_BUF_SIZE)

/* ---------- Utility ---------- */
static inline uint32_t pack4_be(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return ((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | (uint32_t)d;
}

static inline uint16_t u16_le(uint8_t lo, uint8_t hi)
{
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

/* Tokens */
enum {
    TOK_EXT_HDR = 0xF4F3F2F1u,
    TOK_CMD_HDR = 0xFDFCFBFAu
};

static const uint8_t EXT_FTR[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };
static const uint8_t CMD_FTR[4] = { 0x04, 0x03, 0x02, 0x01 };

/* ---------- Parser states ---------- */
typedef enum {
    ST_IDLE = 0,

    /* Minimal frame */
    ST_MIN_STATE,
    ST_MIN_DIST0,
    ST_MIN_DIST1,
    ST_MIN_FOOTER,

    /* Extended frame */
    ST_EXT_LEN0,
    ST_EXT_LEN1,
    ST_EXT_PAYLOAD,
    ST_EXT_FTR0, ST_EXT_FTR1, ST_EXT_FTR2, ST_EXT_FTR3,

    /* Command frame */
    ST_CMD_LEN0,
    ST_CMD_LEN1,
    ST_CMD_PAYLOAD,
    ST_CMD_FTR0, ST_CMD_FTR1, ST_CMD_FTR2, ST_CMD_FTR3
} parser_state_t;

void process_serial(void)
{
    char rx[RX_CHUNK_SIZE];

    parser_state_t st = ST_IDLE;

    /* Rolling window for header detection */
    uint32_t w4 = 0;
    unsigned w4_fill = 0;

    /* Minimal temps */
    uint8_t  min_state = 0;
    uint16_t min_dist  = 0;

    /* Extended payload assembly */
    uint16_t ext_len = 0;
    uint8_t  ext_buf[EXT_PAYLOAD_BUF_SIZE];
    uint16_t ext_have = 0;

    /* Command payload assembly */
    uint16_t cmd_len = 0;
    uint8_t  cmd_buf[CMD_PAYLOAD_BUF_SIZE];
    uint16_t cmd_have = 0;

    /* Footer match index for strict sequential verification */
    uint8_t ftr_bad = 0;

    do {
        size_t r = uart_get_chars(rx, (unsigned)sizeof(rx));
        if (r == 0) continue;

        for (unsigned i = 0; i < r; ++i) {
            uint8_t b = (uint8_t)rx[i];

            /* Update rolling 4-byte window (for header detection only) */
            if (w4_fill < 4) {
                w4 = (w4 << 8) | b;
                w4_fill++;
            } else {
                w4 = (w4 << 8) | b;
            }

            switch (st) {
                /* ---------------- IDLE: detect starts ---------------- */
                case ST_IDLE:
                    if (b == 0x6E) {
                        st = ST_MIN_STATE;
                        break;
                    }
                    if (w4_fill >= 4 && w4 == (uint32_t)TOK_EXT_HDR) {
                        st = ST_EXT_LEN0;
                        ext_len = 0;
                        ext_have = 0;
                        break;
                    }
                    if (w4_fill >= 4 && w4 == (uint32_t)TOK_CMD_HDR) {
                        st = ST_CMD_LEN0;
                        cmd_len = 0;
                        cmd_have = 0;
                        break;
                    }
                    break;

                /* ---------------- Minimal frame ---------------- */
                case ST_MIN_STATE:
                    min_state = b;
                    st = ST_MIN_DIST0;
                    break;

                case ST_MIN_DIST0:
                    min_dist = (uint16_t)b;
                    st = ST_MIN_DIST1;
                    break;

                case ST_MIN_DIST1:
                    min_dist |= (uint16_t)b << 8;
                    st = ST_MIN_FOOTER;
                    break;

                case ST_MIN_FOOTER:
                    if (b == 0x62) {
                        handle_minimal_detection(min_state, min_dist);
                    }
                    st = ST_IDLE;
                    break;

                /* ---------------- Extended frame ---------------- */
                case ST_EXT_LEN0:
                    ext_len = b;
                    st = ST_EXT_LEN1;
                    break;

                case ST_EXT_LEN1:
                    ext_len |= (uint16_t)b << 8;
                    /* Sanity */
                    if (ext_len == 0 || ext_len > MAX_EXT_PAYLOAD_LEN) {
                        st = ST_IDLE; /* discard */
                    } else {
                        ext_have = 0;
                        st = ST_EXT_PAYLOAD;
                    }
                    break;

                case ST_EXT_PAYLOAD:
                    ext_buf[ext_have++] = b;
                    if (ext_have >= ext_len) {
                        ftr_bad = 0;
                        st = ST_EXT_FTR0;
                    }
                    break;

                case ST_EXT_FTR0:
                    if (b != EXT_FTR[0]) { ftr_bad = 1; }
                    st = ST_EXT_FTR1;
                    break;

                case ST_EXT_FTR1:
                    if (b != EXT_FTR[1]) { ftr_bad = 1; }
                    st = ST_EXT_FTR2;
                    break;

                case ST_EXT_FTR2:
                    if (b != EXT_FTR[2]) { ftr_bad = 1; }
                    st = ST_EXT_FTR3;
                    break;

                case ST_EXT_FTR3:
                    if (b != EXT_FTR[3]) { ftr_bad = 1; }

                    if (!ftr_bad) {
                        /* ext_buf[0..ext_len): [type ...] */
                        uint8_t ftype = ext_buf[0];

                        if (ftype == 0x01) {
                            /* payload: type(1), state(1), dist(2), energy(64)... */
                            if (ext_len >= 1u + 1u + 2u) {
                                uint8_t state = ext_buf[1];
                                uint16_t dist = u16_le(ext_buf[2], ext_buf[3]);
                                handle_extended_detection(state, dist);
                            }
                        } else if (ftype == 0x03) {
                            /* payload: type(1), progress(2) */
                            if (ext_len >= 1u + 2u) {
                                uint16_t prog = u16_le(ext_buf[1], ext_buf[2]);
                                handle_calibration_progress(prog);
                            }
                        } else {
                            /* Unknown type: ignore */
                        }
                    }
                    /* Either way, discard frame and return to IDLE */
                    st = ST_IDLE;
                    break;

                /* ---------------- Command frame ---------------- */
                case ST_CMD_LEN0:
                    cmd_len = b;
                    st = ST_CMD_LEN1;
                    break;

                case ST_CMD_LEN1:
                    cmd_len |= (uint16_t)b << 8;
                    /* Sanity: must at least include cmd(2)+code(2) */
                    if (cmd_len < 2u || cmd_len > MAX_CMD_PAYLOAD_LEN) {
                        st = ST_IDLE; /* discard */
                    } else {
                        cmd_have = 0;
                        st = ST_CMD_PAYLOAD;
                    }
                    break;

                case ST_CMD_PAYLOAD:
                    cmd_buf[cmd_have++] = b;
                    if (cmd_have >= cmd_len) {
                        ftr_bad = 0;
                        st = ST_CMD_FTR0;
                    }
                    break;

                case ST_CMD_FTR0:
                    if (b != CMD_FTR[0]) { ftr_bad = 1; }
                    st = ST_CMD_FTR1;
                    break;

                case ST_CMD_FTR1:
                    if (b != CMD_FTR[1]) { ftr_bad = 1; }
                    st = ST_CMD_FTR2;
                    break;

                case ST_CMD_FTR2:
                    if (b != CMD_FTR[2]) { ftr_bad = 1; }
                    st = ST_CMD_FTR3;
                    break;

                case ST_CMD_FTR3:
                    if (b != CMD_FTR[3]) { ftr_bad = 1; }

                    if (!ftr_bad) {
                        uint16_t cmd_word = u16_le(cmd_buf[0], cmd_buf[1]);
                        uint16_t resp     = u16_le(cmd_buf[2], cmd_buf[3]);
                        const uint8_t *data = (cmd_len > 4u) ? &cmd_buf[4] : (const uint8_t *)0;
                        size_t data_len = (cmd_len > 4u) ? (size_t)(cmd_len - 4u) : 0;

                        handle_command_frame(cmd_word, resp, data, data_len);
                    }
                    st = ST_IDLE;
                    break;

                default:
                    st = ST_IDLE;
                    break;
            }
        }
    } while (0);
}

/* ---------- Handler stubs (replace) ---------- */

#include "ld2410s_defs.h"
enum {
    CFG_IDLE,
    CFG_BEGIN,
    CFG_SET_PARAMS,
    CFG_END,
};

enum {
    CAL_IDLE,
    CAL_BEGIN,
    CAL_REQUEST_AUTO_THRESHOLD,
    CAL_END_REQUEST,
    CAL_WAIT_CALIBRATION,
    CAL_BEGIN_READ_GATES,
    CAL_READ_GATES_TRIGGER,
    CAL_READ_GATES_HOLD,
    CAL_READ_GATES_SNR,
    CAL_END,
};

enum _reset_sequence {
    RESET_SEQ_IDLE,
    RESET_ENABLE_CONF,
    RESET_TRIGGER,
    RESET_HOLD,
    RESET_SNR,
    RESET_DISABLE_CONF,
};


static uint8_t configStep = CFG_IDLE;
static uint8_t nextConfigStep = CFG_IDLE;

static uint8_t calibrationStep = CAL_IDLE;
static uint8_t nextCalibrationStep = CAL_IDLE;

static uint8_t reset_seq_step = RESET_SEQ_IDLE;
static uint8_t reset_next_step = RESET_SEQ_IDLE;

static void handle_minimal_detection(uint8_t state, uint16_t distance)
{
    switch (state) {
    case 0:
    case 1:
        ld24xx_MotionDetected = 0;
        ld24xx_Distance = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, ld24xx_MotionDetected));
        break;
    case 2:
    case 3:
        ld24xx_Distance = distance;
        ld24xx_MotionDetected = 1;
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, ld24xx_MotionDetected));
        break;
    default:
        break;
    }

}

static void handle_extended_detection(uint8_t state, uint16_t distance)
{
    handle_minimal_detection(state, distance);
}


static void handle_version_frame(uint16_t ack_code, const uint8_t *data, size_t data_len)
{
    if (data_len >= 4) {
        uint16_t minor_v = u16_le(data[0], data[1]);
        uint16_t patch_v = u16_le(data[2], data[3]);
        ld24xx_fw_version = (ack_code << 16) | // Major version
            (minor_v << 8) | patch_v;
        ESP_LOGI(RX_TASK_TAG, "FW Ver: 0x%06x", ld24xx_fw_version);
    }
    light_driver_set_power(0, 0, 0);
}

static uint16_t gates[LD2410S_NUM_GATES];
static uint16_t gate_indexes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static char printbuf[128];

static void handle_thresholds_frame(const uint8_t *data, size_t data_len, uint16_t *gates, uint16_t next_cmd, uint16_t cmd)
{
    unsigned i, g;
    printbuf[0] = 0;
    uint32_t l = 0;
    for (i = 0, g = 0; i < data_len; i+=4, g++) {
        gates[g] = pack4_be(data[i+3], data[i+2], data[i+1], data[i]);
        l += sprintf(&printbuf[l], " %02u", gates[g]);
    }
    ESP_LOGI(RX_TASK_TAG, "Table %x: %s", cmd, printbuf);
//    if (0 != next_cmd)
//        send_command(next_cmd, (uint8_t *)&gate_indexes, sizeof(gate_indexes));
}

static void handle_command_frame(uint16_t cmd,
                                 uint16_t ack_code,
                                 const uint8_t *data,
                                 size_t data_len)
{
    ld24xx_comm_state = COMM_STATE_READY;

//    if (data_len < 4)
//        return;

    ESP_LOGI(RX_TASK_TAG, "CMD 0x%x (%u bytes): ack=%u", cmd, data_len, ack_code);

//    if (ack_code != 0)
//        return;

    switch (cmd) {
        case CMD_GET_VERSION | CMD_ACK:
            handle_version_frame(ack_code, data, data_len);
            break;
            /* Configuration mode successfully enabled or disabled */
        case CMD_ENABLE_CONFIG | CMD_ACK:
            req_calibration_time = 0;
            if (data_len >= 4)
                ESP_LOGI(RX_TASK_TAG, "Proto ver: %d bufsize: %d", u16_le(data[0], data[1]), u16_le(data[2], data[3]));
             __attribute__((__fallthrough__)); /* fall through */
        case CMD_SET_MODE | CMD_ACK:
        case CMD_DISABLE_CONFIG | CMD_ACK:
        case CMD_SET_PARAMS | CMD_ACK:
            /* Parameter was successfully set, continue to the next step. */
            reset_seq_step = reset_next_step;
            configStep = nextConfigStep;
            if (CAL_READ_GATES_TRIGGER != calibrationStep)
                calibrationStep = nextCalibrationStep;
            break;

        case CMD_START_CALIBRATION | CMD_ACK:
            calibrationStep = nextCalibrationStep;
            break;

        case CMD_READ_TRIGGER | CMD_ACK:
            calibrationStep = nextCalibrationStep;
            handle_thresholds_frame(data, data_len, gates, CMD_READ_HOLD, cmd);
            ld24xx_gate0 = gates[0];
            ld24xx_gate3 = gates[3];
            break;

        case CMD_READ_SNR | CMD_ACK:
            calibrationStep = nextCalibrationStep;
            handle_thresholds_frame(data, data_len, gates, 0, cmd);
//            send_command(CMD_DISABLE_CONFIG, NULL, 0);
            break;

        case CMD_READ_HOLD | CMD_ACK:
            calibrationStep = nextCalibrationStep;
            handle_thresholds_frame(data, data_len, gates, CMD_READ_SNR, cmd);
            ld24xx_gate0 = (ld24xx_gate0 + gates[0]) / 2;
            ld24xx_gate3 = (ld24xx_gate3 + gates[3]) / 2;
            break;

        case CMD_WRITE_TRIGGER | CMD_ACK:
        case CMD_WRITE_SNR | CMD_ACK:
        case CMD_WRITE_HOLD | CMD_ACK:
            reset_seq_step = reset_next_step;
            break;
        default:
            break;
    }

    ESP_LOGI(RX_TASK_TAG, "States: %d %d", configStep, calibrationStep);
}


static void handle_calibration_progress(uint16_t progress)
{
    ld24xx_calibrationTimer = ((100 - progress) * req_calibration_time) / 100;
    if (100 <= progress) {
//        calibrationStep = nextCalibrationStep;
        ld24xx_comm_state = COMM_STATE_READY;
        req_calibration_time = 0;
        calibrationStep = nextCalibrationStep;
        vTaskDelay(pdMS_TO_TICKS(5000));
//        send_command(CMD_ENABLE_CONFIG, NULL, 0);
//        vTaskDelay(pdMS_TO_TICKS(500));
//        send_command(CMD_READ_TRIGGER, (uint8_t *)&gate_indexes, sizeof(gate_indexes));
    }
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

    ld24xx_comm_state = COMM_STATE_WAIT_ACK;
    ESP_LOGI(RX_TASK_TAG, "Send 0x%x (%lu bytes)", command, frame_len);
}

static uint16_t sensor_parameters[] = {
    PARAM_TIMEOUT,
    0, 0x00,
    PARAM_GATE_FAR,
    15, 0x00,
    PARAM_GATE_NEAR,
    0, 0x00,
    PARAM_STATUS_FREQ,
    60, 0x00,
    PARAM_DISTANCE_FREQ,
    60, 0x00,
    PARAM_RESPONSE_SPEED,
    5, 0x00,
};


static void update_configuration(void)
{
    int32_t fargate;

    switch(configStep) {
        case CFG_BEGIN:
            ESP_LOGI(RX_TASK_TAG, "Begin config");
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;

    /* Update the parameter */
        case CFG_SET_PARAMS:
            sensor_parameters[1] = ld24xx_Timeout & 0xffff;
            fargate = (ld24xx_MaxDistance / 10);
            if (fargate > 0xf) fargate = 0xf;
            if (fargate < 1) fargate = 1;
            sensor_parameters[4] = fargate;
            send_command(CMD_SET_PARAMS, (uint8_t *)&sensor_parameters, sizeof(sensor_parameters));
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

//    if (0 != req_calibration_time)
//        return;

    switch (calibrationStep) {
        case CAL_BEGIN:
            ESP_LOGI(RX_TASK_TAG, "Begin calibration");
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;
        case CAL_REQUEST_AUTO_THRESHOLD:
            req_calibration_time = ld24xx_calibrationTimer;
            command_data[0] = ld24xx_trigger_threshold / 10;
            command_data[1] = ld24xx_keep_threshold / 10;
            command_data[2] = req_calibration_time;
            send_command(CMD_START_CALIBRATION, (uint8_t *)&command_data, sizeof(command_data));
            break;
        case CAL_END_REQUEST:
            send_command(CMD_DISABLE_CONFIG, NULL, 0);
            break;
        case CAL_WAIT_CALIBRATION:
//            if (0 != req_calibration_time)
//                return;
//            ESP_LOGI(RX_TASK_TAG, "End calibration wait");
            break;

        case CAL_BEGIN_READ_GATES:
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;
        case CAL_READ_GATES_TRIGGER:
            send_command(CMD_READ_TRIGGER, (uint8_t *)&gate_indexes, sizeof(gate_indexes));
            break;
        case CAL_READ_GATES_HOLD:
            send_command(CMD_READ_HOLD, (uint8_t *)&gate_indexes, sizeof(gate_indexes));
            break;
        case CAL_READ_GATES_SNR:
            send_command(CMD_READ_SNR, (uint8_t *)&gate_indexes, sizeof(gate_indexes));
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

/* Temporary buffer to assemble command data - 6 bytes per gate */
static uint16_t threshold_table_buffer[LD2410S_NUM_GATES * 3];

static uint8_t factory_trigger[][LD2410S_NUM_GATES] = {
    { 48, 42, 36, 34, 32, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31 },
    { 34, 33, 36, 34, 32, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31 },
    { 52, 44, 40, 38, 34, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33 },
    { 30, 28, 31, 29, 27, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25 },
    { 30, 28, 31, 29, 27, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25 },
    };
static uint8_t factory_hold[][LD2410S_NUM_GATES]    = {
    { 45, 42, 33, 32, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28 },
    { 32, 31, 36, 33, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28 },
    { 48, 45, 36, 35, 31, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 },
    { 26, 24, 27, 26, 25, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22 },
    { 26, 24, 27, 26, 25, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22 },
    };
static uint8_t factory_snr[][LD2410S_NUM_GATES]     = {
    { 51, 50, 30, 28, 25, 25, 25, 25, 25, 25, 25, 25, 25, 22, 22, 22 },
    { 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10 },
    { 51, 50, 40, 36, 36, 36, 36, 36, 36, 36, 36, 36, 28, 28, 28, 28 },
    { 51, 50, 30, 28, 25, 25, 25, 25, 25, 25, 25, 25, 25, 22, 22, 22 },
    { 33, 32, 25, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 20, 20, 20 },
    };

static inline void cmd_write_threshold_table(uint16_t cmd, int32_t req)
{
    if (req < 0) req = 0;

/* Select table based on the command and request code */
     uint8_t *tbl = NULL;

    switch (cmd) {
        case CMD_WRITE_TRIGGER:
            if (req >= lengthof(factory_trigger))
                req = 0;
            tbl = factory_trigger[req];
            break;
        case CMD_WRITE_HOLD:
            if (req >= lengthof(factory_hold))
                req = 0;
            tbl = factory_hold[req];
            break;
        case CMD_WRITE_SNR:
            if (req >= lengthof(factory_snr))
                req = 0;
            tbl = factory_snr[req];
            break;
        default:
            return;
    }

    if (!tbl) return;
    ESP_LOGI(RX_TASK_TAG, "Cmd %x Tbl %d", cmd, req);

/* Write gate threshold tables */
    unsigned i;
    uint16_t *p;
    for (i = 0, p = threshold_table_buffer; i < LD2410S_NUM_GATES; i++, p += 3) {
        p[0] = i;
        p[1] = tbl[i];
        p[2] = 0;
    }
    send_command(cmd, (uint8_t *)&threshold_table_buffer, sizeof(threshold_table_buffer));
}

static void reset_thresholds(void)
{
    if (0 == resetRequest)
        return;

    switch (reset_seq_step) {

        case RESET_ENABLE_CONF:
            ESP_LOGI(RX_TASK_TAG, "Reset sensor tables");
            send_command(CMD_ENABLE_CONFIG, NULL, 0);
            break;

        case RESET_TRIGGER:
            cmd_write_threshold_table(CMD_WRITE_TRIGGER, resetRequest - 1);
            break;
        case RESET_HOLD:
            cmd_write_threshold_table(CMD_WRITE_HOLD, resetRequest - 1);
            break;
        case RESET_SNR:
            cmd_write_threshold_table(CMD_WRITE_SNR, resetRequest - 1);
            break;

        case RESET_DISABLE_CONF:
            send_command(CMD_DISABLE_CONFIG, NULL, 0);
            resetRequest = 0;
            break;
        default:
            break;
    }

    reset_next_step = reset_seq_step + 1;
    if (reset_next_step > RESET_DISABLE_CONF)
        reset_next_step = RESET_SEQ_IDLE;
}


void ld24xx_task(void *arg)
{
/* Indicate red color until the sensor is detected */
    light_driver_init(255, 0, 0);
    init_gpio();
    init_uart();

/* This delay is needed for UART to become active */
    vTaskDelay(pdMS_TO_TICKS(4000));

    init_timer(&timer, timer_resolution);

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint64_t clock_now = 0, clock_prev = 0;
    uint64_t ack_timer = 0;
    uint64_t cfg_timer = UINT_MAX;

    int32_t prev_timeout = ld24xx_Timeout;
    int32_t prev_maxdist = ld24xx_MaxDistance;

    while (1) {
        clock_now = clock();
        uint64_t delta = clock_now - clock_prev;
        clock_prev = clock_now;

        cfg_timer += delta;
        ack_timer += delta;

    /* Perform init/config/monitoring tasks */
        if (COMM_STATE_READY == ld24xx_comm_state) {
            ack_timer = 0;

            if (ld24xx_fw_version < 0) {
                send_command(CMD_GET_VERSION, NULL, 0);
                continue;
            }

            if (CFG_IDLE == configStep && CAL_IDLE == calibrationStep && RESET_SEQ_IDLE == reset_seq_step) {
                if (cfg_timer > timer_resolution * 10000 ||
                    prev_timeout != ld24xx_Timeout ||
                    prev_maxdist != ld24xx_MaxDistance ) {
//                    ESP_LOGI(RX_TASK_TAG, "Config start");
                    configStep = CFG_BEGIN;
                    cfg_timer = 0;
                    prev_timeout = ld24xx_Timeout;
                    prev_maxdist = ld24xx_MaxDistance;
                } else if (0 != ld24xx_calibrationTimer && 0 == req_calibration_time) {
                    calibrationStep = CAL_BEGIN;
                } else if (0 != resetRequest) {
                    reset_seq_step = RESET_ENABLE_CONF;
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

            if (RESET_SEQ_IDLE != reset_seq_step) {
                reset_thresholds();
            }

        } else {
    /* Ack not received within timeout period - exit configuration mode */

            if (ack_timer > ack_wait_timeout * timer_resolution) {
                ld24xx_comm_state = COMM_STATE_READY;
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

#endif
