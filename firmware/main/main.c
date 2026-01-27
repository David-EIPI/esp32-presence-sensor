/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "main.h"
#include "clock.h"
#include "ld24xx.h"
#include "ota.h"


static const char *TAG = "APP_MAIN";

//static gptimer_handle_t timer = 0;
//static const uint32_t timer_resolution = 10 * 1000;

//TaskHandle_t dimmerTaskHandle = NULL;

static temperature_sensor_handle_t temp_handle = NULL;
static float temperature_sensor_value = 0;

static const int ZB_INIT_FAIL_COUNT_TO_REBOOT = 60;
static const int ZB_ZDO_FAIL_COUNT_TO_REBOOT = 100;
static int zb_fail_count = 0;


/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}


const char ZB_STORAGE_NAMESPACE[] = "zb_storage";

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
//    uint8_t min_lqi;
//    ESP_LOGI(TAG, ">sig %u", sig_type);

    switch (sig_type) {
    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGI(TAG, "%s, status: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        esp_zb_secur_network_min_join_lqi_set(0);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }

            zb_fail_count = 0;
        } else {
            /* commissioning failed */

            if (ZB_INIT_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
                ESP_LOGI(TAG, "ZB init has failed too many times. Restarting.");
                esp_restart();
                break;
            }

            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            zb_fail_count += 1;
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Leave and network steering initiated.");
        esp_zb_zcl_reset_nvram_to_factory_default();
        esp_zb_factory_reset();
        esp_zb_bdb_reset_via_local_action();
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        if (ZB_ZDO_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
            ESP_LOGI(TAG, "ZDO device unavailable. Restarting.");
            esp_restart();
            break;
        }
        zb_fail_count += 1;
        __attribute__ ((fallthrough));
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


/* Setup I/O attributes.
  Analog and binary input cluser is used to report sensor readings.
  Analog output clusters are used to configure device parameters.
 */

static struct {
    char     description[16];
    float    min_present;
    float    max_present;
    float    present_value;
    float    resolution;
    uint16_t status_flags;
    uint16_t units;
    uint32_t app_type;
    volatile int32_t *value_ptr;
    char     nvs_key[8];
} analog_output_attr_values[] = {
    {
    .description = "\x07" "Timeout",
    .min_present = 0,
    .max_present = 3600,
    .present_value = 20,
    .resolution = 1,
    .status_flags = 0,
    .units = 73, /* 73 = seconds */
    .app_type = 14 << 16, /* 14 = timer */
//    .value_ptr = &motionTimeout,
    .value_ptr = &ld24xx_Timeout,
    .nvs_key = "AO_00",
    },

    {
    .description = "\x0e" "Distance range",
    .min_present = 1,
    .max_present = 1000,
    .present_value = 200,
    .resolution = 1,
    .status_flags = 0,
    .units = 118, /* 118 = cm */
    .app_type = 2 << 16, /* 2 = Gauge */
    .value_ptr = &ld24xx_MaxDistance,
    .nvs_key = "AO_01",
    },

    {
    .description = "\x0f" "Trigger thresh.",
    .min_present = 10,
    .max_present = 100,
    .present_value = 50,
    .resolution = 1,
    .status_flags = 0,
    .units = 95, /* No units */
    .app_type = 12 << 16, /* counter */
    .value_ptr = &ld24xx_trigger_threshold,
    .nvs_key = "AO_02",
    },

    {
    .description = "\x0e" "Hold threshold",
    .min_present = 10,
    .max_present = 100,
    .present_value = 30,
    .resolution = 1,
    .status_flags = 0,
    .units = 95, /*No units */
    .app_type = 12 << 16, /* counter */
    .value_ptr = &ld24xx_keep_threshold,
    .nvs_key = "AO_03",
    },

};

static struct {
    char     description[16];
    float    min_present;
    float    max_present;
    float    present_value;
    float    resolution;
    uint16_t status_flags;
    uint16_t units;
    uint32_t app_type;
    volatile int32_t *value_ptr;
    int32_t prev_value;
    int16_t endpoint;
} analog_input_attr_values[] = {
    {
    .description = "\x08" "Distance",
    .min_present = 0,
    .max_present = 1e3,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 118, /* 118 = cm */
    .app_type = 0xff, /* Other - disable unit override in zigpy */
    .value_ptr = &ld24xx_Distance,
    .prev_value = 0,
    .endpoint = -1,
    },
    {
    .description = "\x0b" "Calibration",
    .min_present = 0,
    .max_present = 1000,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 73, /* 73 = seconds */
    .app_type = 14 >> 16, /* timer */
    .value_ptr = &ld24xx_calibrationTimer,
    .prev_value = 0,
    .endpoint = -1,
    },
    {
    .description = "\x06" "Gate 0",
    .min_present = 0,
    .max_present = 1e7,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 95, /* 95 = no unit */
    .app_type = 0xff >> 16,
    .value_ptr = &ld24xx_gate0,
    .prev_value = 0,
    .endpoint = -1,
    },
    {
    .description = "\x06" "Gate 3",
    .min_present = 0,
    .max_present = 1e7,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 95, /* 95 = no unit */
    .app_type = 0xff >> 16,
    .value_ptr = &ld24xx_gate3,
    .prev_value = 0,
    .endpoint = -1,
    },
};

#define ANALOG_INPUT_ATTR_FIELD_OFFSET(field) (offsetof(typeof(analog_input_attr_values[0]), field))

static struct {
    int id;
    int offs;
} analog_input_attr_offset[] = {
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID      , ANALOG_INPUT_ATTR_FIELD_OFFSET(description),   },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, ANALOG_INPUT_ATTR_FIELD_OFFSET(max_present),   },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, ANALOG_INPUT_ATTR_FIELD_OFFSET(min_present),   },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID    , ANALOG_INPUT_ATTR_FIELD_OFFSET(present_value), },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID       , ANALOG_INPUT_ATTR_FIELD_OFFSET(resolution),    },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_STATUS_FLAGS_ID     , ANALOG_INPUT_ATTR_FIELD_OFFSET(status_flags),  },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, ANALOG_INPUT_ATTR_FIELD_OFFSET(units),         },
/* Home Assistant/ZHA wrongly overrides units based on the application type attribute, so disable it for now */
//    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID , ANALOG_INPUT_ATTR_FIELD_OFFSET(app_type),      }
};


static esp_err_t nvs_save_int_attribute(int32_t value, const char *key)
{
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_OK;

//    ESP_LOGI(TAG, "Saving attribute: %s = %ld", key, value);
    ESP_RETURN_ON_ERROR(nvs_open(ZB_STORAGE_NAMESPACE, NVS_READWRITE, &handle), TAG, "Error opening NVS handle!");

    err = nvs_set_i32(handle, key, value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to NVS (%s)!", esp_err_to_name(err));

    } else {

        err = nvs_commit(handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to commit to NVS (%s)!", esp_err_to_name(err));
        }
    }

    nvs_close(handle);
    return err;
}

static esp_zb_attribute_list_t * create_analog_input_cluster(unsigned idx, int16_t ep_idx)
{

    esp_zb_attribute_list_t *esp_zb_analog_input_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    int nattrs = 0;
    if (esp_zb_analog_input_cluster != NULL) {
        int i;

        for (i = 0; i < lengthof(analog_input_attr_offset); i++) {

	    esp_err_t err = esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster, analog_input_attr_offset[i].id,
	        ((void*)&analog_input_attr_values[idx]) + analog_input_attr_offset[i].offs);
	    if (ESP_OK == err) {
	        nattrs += 1;
	        analog_input_attr_values[idx].endpoint = ep_idx;
	    }
        }
    }

    ESP_LOGI(TAG, "Analog input cluster %u created. Attrs: %d", idx, nattrs);
    return esp_zb_analog_input_cluster;
}


#define ANALOG_ATTR_FIELD_OFFSET(field) (offsetof(typeof(analog_output_attr_values[0]), field))

static struct {
    int id;
    int offs;
    int access;
} analog_attr_offset[] = {
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID      , ANALOG_ATTR_FIELD_OFFSET(description),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(max_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(min_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID    , ANALOG_ATTR_FIELD_OFFSET(present_value), .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID       , ANALOG_ATTR_FIELD_OFFSET(resolution),    .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_STATUS_FLAGS_ID     , ANALOG_ATTR_FIELD_OFFSET(status_flags),  .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, ANALOG_ATTR_FIELD_OFFSET(units),         .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_APPLICATION_TYPE_ID , ANALOG_ATTR_FIELD_OFFSET(app_type),      .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY }
};

/* Setup AnalogOutput clusters and add them to the endpoints starting from the provided index. */
static esp_zb_attribute_list_t * create_analog_output_cluster(unsigned idx, nvs_handle_t handle)
{
    esp_zb_attribute_list_t *esp_zb_analog_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT);
    int nattrs = 0;
    if (esp_zb_analog_cluster != NULL) {

/*  Attempt to find a saved attribute value */
        int32_t saved_value = 0;
        if (handle && ESP_OK == nvs_get_i32(handle, analog_output_attr_values[idx].nvs_key, &saved_value)) {
            *analog_output_attr_values[idx].value_ptr = saved_value;
            ESP_LOGI(TAG, "Loaded attribute: %s = %ld", analog_output_attr_values[idx].nvs_key, saved_value);
        }
        analog_output_attr_values[idx].present_value = *analog_output_attr_values[idx].value_ptr;

        int i;
        for (i = 0; i < lengthof(analog_attr_offset); i++) {
	    esp_err_t err = esp_zb_analog_output_cluster_add_attr(esp_zb_analog_cluster, analog_attr_offset[i].id,
	        ((void*)&analog_output_attr_values[idx]) + analog_attr_offset[i].offs );
	    if (ESP_OK == err)
	        nattrs += 1;
        }
    }

    ESP_LOGI(TAG, "Analog output cluster %u created. Attrs: %d", idx, nattrs);
    return esp_zb_analog_cluster;
}


volatile uint8_t motion_detected = 0;
volatile uint8_t micro_motion_detected = 0;

static struct {
    char     description[16];
    uint8_t *value_ptr;
    uint8_t prev_value;
    int16_t endpoint;
} binary_input_attr_values[] = {
    {
        .description = "\x0f" "Motion detected",
        .value_ptr = &ld24xx_MotionDetected,
        .prev_value = 0,
        .endpoint = -1,
    },

    {
        .description = "\x0f" "PS Interference",
        .value_ptr = &ld24xx_Interference,
        .prev_value = 0,
        .endpoint = -1,
    },

};

static esp_zb_attribute_list_t * create_binary_input_cluster(int idx, int16_t ep_idx)
{
    esp_zb_binary_input_cluster_cfg_t cfg = { false, 0 };
    esp_zb_attribute_list_t * binary_input_cluster = esp_zb_binary_input_cluster_create(&cfg);

    if (NULL != binary_input_cluster) {

        ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(
            binary_input_cluster,
            ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
            binary_input_attr_values[idx].description
        ));
        ESP_ERROR_CHECK(esp_zb_cluster_add_attr(binary_input_cluster,
	    ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
	    ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
	    ESP_ZB_ZCL_ATTR_TYPE_BOOL,
	    ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
	    binary_input_attr_values[idx].value_ptr
	));

        binary_input_attr_values[idx].endpoint = ep_idx;
    }

    return binary_input_cluster;
}

/* Setup Temperature Sensor cluster */
static esp_zb_attribute_list_t * create_temperature_cluster(void)
{
    esp_zb_temperature_meas_cluster_cfg_t cfg = { 0, -10 * 100, 80 * 100 };
    esp_zb_attribute_list_t * temperature_measurement_cluster = esp_zb_temperature_meas_cluster_create(&cfg);

    return temperature_measurement_cluster;
}

/* Setup Basic cluster */
static esp_zb_attribute_list_t * create_basic_cluster(char *manufacturer)
{
    uint8_t zero = 0;
    uint8_t version = 3;

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);

    if (esp_zb_basic_cluster != NULL) {
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &version);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &zero);
    }
    return esp_zb_basic_cluster;
}

static  esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = 0, //HA_ANALOG_INPUT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 0,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 0,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

static struct {
    uint16_t endpoint_id, cluster_id, attr_id, delta;
} reportable_attr_config[] = {
    {
        HA_DISTANCE_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        1
    },
    {
        HA_MOTION_BINARY_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        1
    },
    {
        HA_MOTION_BINARY_ENDPOINT + 1,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        1
    }
};

static esp_err_t setup_reporting(void)
{
    unsigned i;
 /* Config the reporting info  */
    for (i = 0; i < lengthof(reportable_attr_config); i++) {
        reporting_info.ep         = reportable_attr_config[i].endpoint_id;
        reporting_info.cluster_id = reportable_attr_config[i].cluster_id;
        reporting_info.attr_id    = reportable_attr_config[i].attr_id;
        reporting_info.u.send_info.delta.u16  = reportable_attr_config[i].delta;
        ESP_RETURN_ON_ERROR(esp_zb_zcl_update_reporting_info(&reporting_info), TAG, "Error setting reporing for %d.%d.%d",
            reporting_info.ep, reporting_info.cluster_id, reporting_info.attr_id);
    }
    ESP_LOGI(TAG, "Setup reporting %u attributes", i);
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

/* Process analog output clusters */
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT
        && message->attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID
        && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_SINGLE)
    {
        uint8_t endpoint = message->info.dst_endpoint;
        if (endpoint >= HA_FIRST_ENDPOINT && endpoint < HA_FIRST_ENDPOINT + lengthof(analog_output_attr_values)) {
            if (message->attribute.data.value) {
                float value = *(float *)message->attribute.data.value;
                int32_t intvalue = (int32_t)((value / analog_output_attr_values[endpoint - HA_FIRST_ENDPOINT].resolution) + 0.5);
                * analog_output_attr_values[endpoint - HA_FIRST_ENDPOINT].value_ptr = intvalue;
                nvs_save_int_attribute(intvalue, analog_output_attr_values[endpoint - HA_FIRST_ENDPOINT].nvs_key);
            }
        }
    }


/* Process analog input clusters that allow value modification */
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT
        && message->attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID
        && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_SINGLE)
    {
        uint8_t endpoint = message->info.dst_endpoint;
        if (endpoint >= HA_FIRST_ENDPOINT && endpoint < HA_FIRST_ENDPOINT + lengthof(analog_input_attr_values)
                && analog_input_attr_values[endpoint - HA_FIRST_ENDPOINT].value_ptr) {
            if (message->attribute.data.value) {
                float value = *(float *)message->attribute.data.value;
                int32_t intvalue = (int32_t)((value / analog_input_attr_values[endpoint - HA_FIRST_ENDPOINT].resolution) + 0.5);
                * analog_input_attr_values[endpoint - HA_FIRST_ENDPOINT].value_ptr = intvalue;
            }
        }
    }


    return ret;
}

static esp_err_t zb_factory_reset_handler(esp_zb_zcl_basic_reset_factory_default_message_t *message)
{
//    resetRequest = 1;
    ESP_LOGW(TAG, "Receive factory reset %u:%u", message->info.dst_endpoint, message->info.cluster);
    resetRequest = message->info.dst_endpoint;
//        esp_restart();
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_BASIC_RESET_TO_FACTORY_RESET_CB_ID:
        ret = zb_factory_reset_handler((esp_zb_zcl_basic_reset_factory_default_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void setup_endpoints(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err;

    err = nvs_open(ZB_STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        handle = 0;
    }

    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();

    int a_idx, ep_idx;
    for (a_idx = 0, ep_idx = HA_FIRST_ENDPOINT; a_idx < lengthof(analog_output_attr_values); a_idx++, ep_idx++ ) {

        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

//        if (a_idx == 0) {
//        if (a_idx < lengthof(analog_output_attr_values) - 1) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
                cluster_list,
                create_basic_cluster(MANUFACTURER_NAME),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
//        }

        if (a_idx < lengthof(analog_input_attr_values)) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(
                cluster_list,
                create_analog_input_cluster(a_idx, ep_idx),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }


        ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(
            cluster_list,
            create_analog_output_cluster(a_idx, handle),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        ));


        if (a_idx < lengthof(binary_input_attr_values)) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(
                cluster_list,
                create_binary_input_cluster(a_idx, ep_idx),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

        if (ep_idx == HA_TEMPERATURE_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
                cluster_list,
                create_temperature_cluster(),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

/* Finally setup a new endpoint */
        esp_zb_endpoint_config_t ep_config = {
            .endpoint = ep_idx,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = HA_DEVICE_ID,
            .app_device_version = 1,
        };


        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
            endpoint_list,
            cluster_list,
            ep_config
        ));

    }

    if (handle)
        nvs_close(handle);

/* Register endpoints */
    ESP_ERROR_CHECK(zb_register_ota_upgrade_client_device(endpoint_list, ep_idx));
    ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list));
    ESP_LOGI(TAG, "Device registered");

    setup_reporting();

}

static void esp_zb_task(void *pvParameters)
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

#if CONFIG_ESP_ZB_TRACE_ENABLE
   esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_DEBUG, ESP_ZB_TRACE_SUBSYSTEM_ZCL | ESP_ZB_TRACE_SUBSYSTEM_NWK | ESP_ZB_TRACE_SUBSYSTEM_TRANSPORT | ESP_ZB_TRACE_SUBSYSTEM_ZDO);
#endif

    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg =
#if defined(ZB_ED_ROLE)
        ESP_ZB_ZED_CONFIG();
#else
        ESP_ZB_ZCZR_CONFIG();
#endif
    esp_zb_init(&zb_nwk_cfg);

    setup_endpoints();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}


static void setup_temperature_sensor(void)
{
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}

static esp_err_t read_temperature_sensor(void)
{
    esp_err_t err = ESP_OK;
    float tsens_out = 0;
    // Enable temperature sensor
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(temp_handle), TAG, "Error enabling temperature sensor!");
    // Get converted sensor data
    ESP_RETURN_ON_ERROR(temperature_sensor_get_celsius(temp_handle, &tsens_out), TAG, "Error reading temperature sensor!");

    temperature_sensor_value = (int16_t)(tsens_out * 100.0f);
    // Disable the temperature sensor if it is not needed and save the power
    ESP_RETURN_ON_ERROR(temperature_sensor_disable(temp_handle), TAG, "Error disabling temperature sensor!");
    return err;
}


static void zb_update_task(void *pvParameters)
{
    static const char *TASK_TAG = "CTRL_TASK";
    uint32_t prev_dist_sensor = 0;
    int16_t prev_temperature = 0;

    setup_temperature_sensor();

    while (1) {

        read_temperature_sensor();
        int have_lock = 0;

        unsigned i;

        for (i = 0; i < lengthof(analog_input_attr_values); i++) {
            if (analog_input_attr_values[i].prev_value != *(analog_input_attr_values[i]).value_ptr
                    && analog_input_attr_values[i].endpoint >= 0) {
                if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
                    have_lock = 1;

                    analog_input_attr_values[i].prev_value = *(analog_input_attr_values[i]).value_ptr;
                    float new_value = analog_input_attr_values[i].prev_value;

                    esp_zb_zcl_set_attribute_val(
                        analog_input_attr_values[i].endpoint,
                        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                        &new_value,
                        false
                    );

                }
            }
        }


        for (i = 0; i < lengthof(binary_input_attr_values); i++) {
            if (binary_input_attr_values[i].prev_value != *(binary_input_attr_values[i]).value_ptr
                    && binary_input_attr_values[i].endpoint >= 0) {
                if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
                    have_lock = 1;

                    binary_input_attr_values[i].prev_value = *(binary_input_attr_values[i]).value_ptr;
                    int32_t new_value = binary_input_attr_values[i].prev_value;

                    esp_zb_zcl_set_attribute_val(
                        binary_input_attr_values[i].endpoint,
                        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                        &new_value,
                        false
                    );
                }
            }
        }

	if (prev_dist_sensor != ld24xx_Distance) {
	    prev_dist_sensor = ld24xx_Distance;
	    ESP_LOGI(TASK_TAG, "R%s: '%lu' %u", ld24xx_eng_mode ? "e":"", prev_dist_sensor, ld24xx_MotionDetected);
	}

	if (prev_temperature != temperature_sensor_value) {
	    if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
	        have_lock = 1;
	        prev_temperature = temperature_sensor_value;

		esp_zb_zcl_set_attribute_val(HA_TEMPERATURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
		    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &prev_temperature, false);
	    }
	}

        if (have_lock) {
	    esp_zb_lock_release();
	    have_lock = 0;
        }

	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
//    init_timer(&timer, timer_resolution);

    xTaskCreate(esp_zb_task, "Zigbee_main", 1024 * 8, NULL, 2, NULL);
    xTaskCreate(ld24xx_task, "uart_rx_task", 1024 * 4, NULL, 1, NULL);

    zb_update_task(NULL);
}
