#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "methodsJS.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include <string.h>

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "main.h"
#include "esp_gatt_common_api.h"

//#define DEFAULT_VREF 1100   //ESP32 ADC reference voltage is 1.1V
#define DEFAULT_VREF 1094

//ADC VARIABLES
static const adc_channel_t channelTAmb = ADC_CHANNEL_2;     //GPIO24 ADC2
static const adc_channel_t channelThermocouple = ADC_CHANNEL_0 ;     //GPIO26 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; //12 bits
static const adc_atten_t atten = ADC_ATTEN_DB_11;       //Range of measurementup to  approx. 2600 mV
static const adc_unit_t unit = ADC_UNIT_2;              //ADC2

static esp_adc_cal_characteristics_t *adc_chars;
#define NO_OF_SAMPLES   50          //Multisampling
#define GAIN_INA122P    145

//GLOBAL VARIABLES
uint32_t thermocouple_uV = 0,  adc_reading_Thermocouple_raw = 0;     //_raw used as a control bit if it is 2 or less thermocuple_temp = 0ºC
uint16_t thermocouple_temp = 0, ambient_temp = 0;
uint16_t reference_temperature;

//GPIO define

#define Buzzer_GPIO    GPIO_NUM_19  //Buzzer
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<Buzzer_GPIO))
//GPIO33:  input, pulled up, interrupt from rising edge.
#define Button_GPIO     GPIO_NUM_18  //Button
#define GPIO_INPUT_PIN_SEL  ((1ULL<<Button_GPIO))
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;

//LEDC define
#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LED_GPIO 			   GPIO_NUM_21
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif
#define LEDC_DUTY         	   (4000)
#define LEDC_FADE_TIME    	   (2000)

TaskHandle_t xHandle;
uint8_t enable = 0;


//BLE START-------------------------------------------------------------------------------------------------------------
#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

//DEFINE APPLICATION PROFILE

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_TC_K_ID                  0x55
#define SAMPLE_DEVICE_NAME          "JSA TC K"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t TC_K_handle_table[HRS_IDX_NB];

uint16_t ble_received;
uint8_t ble_received_c0, ble_received_c1, ble_received_c2;

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

// Profile stored in TC_K_profile_tab

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

//GAP parameters settings

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */ //ITS NEEDED TO ADD 0x0A, 0x09 to the name to be visible
		0x0A, 0x09, 0x4a, 0x53, 0x41, 0x20, 0x54, 0x43, 0x20, 0x4b
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x02, 0x00, 0x12, 0xAC, 0x42, 0x02, 0x64, 0x9D, 0xEC, 0x11, 0x96, 0xBD, 0x62, 0x08, 0x76, 0x46,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

//the longer parameters > 31 bytes are stored in the scan response.
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

//-----------------------------------------------------------------------------------
/* Service */
static const uint16_t GATTS_SERVICE_UUID_ENV      = 0x181A;
static const uint16_t GATTS_CHAR_UUID_TEMP       = 0x2A6E;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_ENV), (uint8_t *)&GATTS_SERVICE_UUID_ENV}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

};

//-----------------------------------------------------------------------------------

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}



static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
	/*!< Minimum advertising interval for undirected and low duty cycle directed advertising.
	    Range: 0x0020 to 0x4000
	    Default: N = 0x0800 (1.28 second)
	    Time = N * 0.625 msec
	    Time Range: 20 ms to 10.24 sec */
    .adv_int_max         = 0x40,
	 /*!< Maximum advertising interval for undirected and low duty cycle directed advertising.
	    Range: 0x0020 to 0x4000
	    Default: N = 0x0800 (1.28 second)
	    Time = N * 0.625 msec
	    Time Range: 20 ms to 10.24 sec */
    .adv_type            = ADV_TYPE_IND,		 /*!< Advertising type */
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,	 /*!< Owner bluetooth device address type */
    .channel_map         = ADV_CHNL_ALL,				/*!< Advertising channel map */
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,	/*!< Advertising filter policy */
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                ble_received_c0 = param->write.value[2];	//char 0
                ble_received_c1 = param->write.value[1]; //TO READ 16bit max
                ble_received_c2 = param->write.value[0];
                if (TC_K_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,TC_K_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, TC_K_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(TC_K_handle_table, param->add_attr_tab.handles, sizeof(TC_K_handle_table));
                esp_ble_gatts_start_service(TC_K_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst TC_K_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};


//GAP EVENT HANDLER


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
        	TC_K_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == TC_K_profile_tab[idx].gatts_if) {
                if (TC_K_profile_tab[idx].gatts_cb) {
                	TC_K_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}



//END BLE


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(io_num == 18){
            	enable += 1;
            	if(enable == 10){
            		enable = 0;
            	}
            }
        }
    }
}


void get_ADC_values(void *pvParameters)
{
    /* Task that acquires n samples of both the ambient and thermocouple sensors and
       filters them applying an exponential moving average. This task provides
       new values every 250ms */

    //Variables for updating the task exactly every 250ms
    uint8_t cnt = 0;
    const TickType_t xDelay250 = pdMS_TO_TICKS(250);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    //Configure ADC
    //adc2_config_width(width);
    adc2_config_channel_atten(channelTAmb, atten);
    adc2_config_channel_atten(channelThermocouple, atten);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    int *raw_p1, *raw_p2;
    int raw_tamb = 0, raw_tk = 0;
    raw_p1 = &raw_tamb;
    raw_p2 = &raw_tk;
    while (1)
    {
        uint32_t adc_reading_Tamb = 0, adc_reading_Thermocouple = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc2_get_raw((adc2_channel_t)channelTAmb,width,raw_p1);
            adc_reading_Tamb += *raw_p1;
            adc2_get_raw((adc2_channel_t)channelThermocouple,width,raw_p2);
            adc_reading_Thermocouple += *raw_p2;
        }
        adc_reading_Tamb /= NO_OF_SAMPLES;
        adc_reading_Thermocouple_raw = adc_reading_Thermocouple / NO_OF_SAMPLES;
        cnt++;
        //Convert adc_reading to voltage in mV
        //Now the ambient temperature is obtained.
        uint32_t voltage_ambient = esp_adc_cal_raw_to_voltage(adc_reading_Tamb, adc_chars);
        thermocouple_uV = ((esp_adc_cal_raw_to_voltage(adc_reading_Thermocouple_raw, adc_chars) * 1000) / GAIN_INA122P);
        ambient_temp = ((voltage_ambient - 500) / 10) - 6; //Value in degrees with respect to zero
        if (cnt == 4)
        {
            printf("Voltage: %d\t Ambient temperature: %d C \n", adc_reading_Tamb, ambient_temp);
            //printf("Raw: %d\tThermocuple voltage: %duV\n", adc_reading_Thermocouple_raw, thermocouple_uV);
            cnt = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xDelay250);
    }
}

void temperature_handle(void *pvParameters)
{
    /* Task that obtains temperatures from the thermocouple applying software compensation of the
    cold junction every 0.5seconds also an alarm is programmed to beep if the temperature surpass
    a set point and a led also indicates if the temperature is close to the setpoint. */
    const TickType_t xDelay1000 = pdMS_TO_TICKS(1000);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t a = 1;
	uint8_t rdm_var;
	char str[8];
	srand(2);
	esp_err_t error_ble_ind;

    //Configure led, configure buzzer pins


    while (1)
    {
        if(adc_reading_Thermocouple_raw < 2)
        {   
            thermocouple_temp = V_to_ktemp(ambient_temp, 0);
        }

        else
        {
            thermocouple_temp = V_to_ktemp(ambient_temp, thermocouple_uV);
        }

        //TEST BUZZER
        if(enable == 1){

        if(a == 1)
        {
            gpio_set_level(Buzzer_GPIO, 1);
            a = 0;
        }

        else
        {
             gpio_set_level(Buzzer_GPIO, 0);
            a = 1;
        }
        }
        else{
        	gpio_set_level(Buzzer_GPIO, 0);
        }
        
        //LED_fade task control depending on thermocouple temperature
        if(enable >= 5 && enable <=8){
        	vTaskResume( xHandle );
        }

       printf("Button pressed: %d\n", enable);
       //printf("Thermocouple temperature: %d C\n",   thermocouple_temp);

       rdm_var = rand() % 100 + 1;
       		sprintf(str, "%d", rdm_var);
       		error_ble_ind = esp_ble_gatts_send_indicate(TC_K_profile_tab[PROFILE_APP_IDX].gatts_if,
       											TC_K_profile_tab[PROFILE_APP_IDX].conn_id,
       											TC_K_handle_table[IDX_CHAR_VAL_A],
       		                                    sizeof(str),
       											&str,
       		                                    false);

       if(error_ble_ind != ESP_OK){
    	   printf("Notify to GATT client has failed \n");
       }

	   ble_received = chartodec(ble_received_c2, ble_received_c1, ble_received_c0);
		//printf("Value received %d \n", ble_received);

       vTaskDelayUntil(&xLastWakeTime, xDelay1000);
    }
}


void led_fade(void *pvParameters)
{
	//LEDC
		/*
	     * Set configuration of timer0 for high speed channels
	     * that will be used by LED Controller
	     */

	ledc_timer_config_t ledc_timer = {
			.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
	        .freq_hz = 5000,                      // frequency of PWM signal
	        .speed_mode = LEDC_HS_MODE,           // timer mode
	        .timer_num =  LEDC_HS_TIMER,           // timer index
	        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
	    };

	ledc_channel_config_t ledc_channel= {
	        .channel    = LEDC_HS_CH0_CHANNEL,
	        .duty       = 0,
	        .gpio_num   = LED_GPIO,
	        .speed_mode = LEDC_HS_MODE,
	        .hpoint     = 0,
	        .timer_sel  = LEDC_HS_TIMER
	    };
	// Set LED Controller with previously prepared configuration
	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel);
    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1) {
    	if(enable < 5){
    		ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    	    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    	    vTaskSuspend( xHandle );
    	}
    	else if(enable >= 5 && enable <=8){
			ledc_set_fade_with_time(ledc_channel.speed_mode,
			ledc_channel.channel, LEDC_DUTY, LEDC_FADE_TIME);
			ledc_fade_start(ledc_channel.speed_mode,
					ledc_channel.channel, LEDC_FADE_NO_WAIT);
			vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
			ledc_set_fade_with_time(ledc_channel.speed_mode,
			ledc_channel.channel, 0, LEDC_FADE_TIME);
			ledc_fade_start(ledc_channel.speed_mode,
					ledc_channel.channel, LEDC_FADE_NO_WAIT);
    	}
    	else{
    		ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, LEDC_DUTY);
    	    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
		}
    	vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
    }

}

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    //Application registration
    ret = esp_ble_gatts_app_register(ESP_TC_K_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    xTaskCreate(get_ADC_values, "get_ADC_values", 2000, NULL, 2, NULL); //If code crashes check stack depth
    xTaskCreate(temperature_handle, "temperature_handle", 2000, NULL, 1, NULL); //If code crashes check stack depth
    xTaskCreate(led_fade, "led_fade", 2000, NULL, 3, &xHandle );

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //Initialization for button with pullup and interrupt at rising edge

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO33
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(Button_GPIO, gpio_isr_handler, (void*) Button_GPIO);
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}
