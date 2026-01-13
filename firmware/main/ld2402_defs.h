#pragma once

enum {
    CMD_GET_VERSION               = 0x0000,  // Read firmware version command
    CMD_ENABLE_CONFIG             = 0x00FF,  // Enable configuration mode
    CMD_DISABLE_CONFIG            = 0x00FE,  // End configuration mode
    CMD_GET_SN_HEX                = 0x0016,  // Read SN (hex format)
    CMD_GET_SN_CHAR               = 0x0011,  // Read SN (character format)
    CMD_GET_PARAMS                = 0x0008,  // Read parameters
    CMD_SET_PARAMS                = 0x0007,  // Set parameters
    CMD_SET_MODE                  = 0x0012,  // Set data output mode
    CMD_START_CALIBRATION         = 0x0009,  // Start automatic threshold generation
    CMD_GET_CALIBRATION_STATUS    = 0x000A,  // Query calibration progress
    CMD_CALIBRATION_INTERFERENCE  = 0x0014,  // Report calibration interference
    CMD_SAVE_PARAMS               = 0x00FD,  // Save parameters to flash
    CMD_AUTO_GAIN                 = 0x00EE,  // Auto gain adjustment
    CMD_AUTO_GAIN_COMPLETE        = 0x00F0,  // Auto gain completion notification
};

enum {
    PARAM_MAX_DISTANCE       = 0x0001,  // Max detection distance
    PARAM_TIMEOUT            = 0x0004,  // Target disappearance delay
    PARAM_POWER_INTERFERENCE = 0x0005,  // Power interference status (read-only)
    PARAM_TRIGGER_THRESHOLD  = 0x0010,  // Motion trigger threshold base (0x0010-0x001F)
    PARAM_STILL_THRESHOLD    = 0x0030,  // Micromotion threshold base (0x0030-0x003F)
    PARAM_SAVE_TRIGGER       = 0x003F,  // Parameter that when written triggers saving data to NVRAM
};

enum {
    MODE_PRODUCTION  = 0x00000064,  // Normal production mode
    MODE_NORMAL      = 0x00000064,  // Alias for production mode
    MODE_CONFIG      = 0x00000001,
    MODE_ENGINEERING = 0x00000004,  // Engineering/debug mode
};


#define LD2402_NUM_GATES 16

/*
 * Convert version string "V#.#.#" into 0xMMmmpp
 * Returns 0 on parse failure.
 */
int32_t version_to_u32(const unsigned char *s, unsigned len)
{
    uint32_t v = 0;
    uint8_t part = 0;     /* 0=major, 1=middle, 2=minor */
    uint32_t acc = 0;      /* decimal accumulator */

    if (!s || len < 1 || (s[0] != 'V' && s[0] != 'v'))
        return 0;

    s++; /* skip 'V' */
    unsigned i = len;
    while (--i && part < 3) {
        if (*s >= '0' && *s <= '9') {
            acc = (uint8_t)(acc * 10u + (uint8_t)(*s - '0'));
            if (acc > 255u) return 0; /* overflow protection */
        } else if (*s == '.') {
            v = (v << 8) | (uint32_t)acc;
            acc = 0;
            part++;
        } else {
            break; /* invalid character */
        }
        s++;
    }

    if (part != 2)  /* must have parsed exactly 3 components */
        return 0;

    /* store minor */
    v = (v << 8) | (uint32_t)acc;

    return v;
}

