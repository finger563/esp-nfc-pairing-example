/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_hid_gap.h"

static const char *TAG = "ESP_HID_GAP";

static bool created_ble_oob_data = false;
static esp_ble_local_oob_data_t ble_oob_sec_data = {0};
static uint8_t TK[16] = {
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
};

static SemaphoreHandle_t bt_hidh_cb_semaphore = NULL;
#define WAIT_BT_CB() xSemaphoreTake(bt_hidh_cb_semaphore, portMAX_DELAY)
#define SEND_BT_CB() xSemaphoreGive(bt_hidh_cb_semaphore)

static SemaphoreHandle_t ble_hidh_cb_semaphore = NULL;
#define WAIT_BLE_CB() xSemaphoreTake(ble_hidh_cb_semaphore, portMAX_DELAY)
#define SEND_BLE_CB() xSemaphoreGive(ble_hidh_cb_semaphore)

#define SIZEOF_ARRAY(a) (sizeof(a) / sizeof(*a))

bool has_created_ble_oob_sec_data() {
    return created_ble_oob_data;
}
esp_ble_local_oob_data_t* get_ble_oob_sec_data_ptr() {
    return &ble_oob_sec_data;
}

static const char *ble_gap_evt_names[] = {"ADV_DATA_SET_COMPLETE",
                                          "SCAN_RSP_DATA_SET_COMPLETE",
                                          "SCAN_PARAM_SET_COMPLETE",
                                          "SCAN_RESULT",
                                          "ADV_DATA_RAW_SET_COMPLETE",
                                          "SCAN_RSP_DATA_RAW_SET_COMPLETE",
                                          "ADV_START_COMPLETE",
                                          "SCAN_START_COMPLETE",
                                          "AUTH_CMPL",
                                          "KEY",
                                          "SEC_REQ",
                                          "PASSKEY_NOTIF",
                                          "PASSKEY_REQ",
                                          "OOB_REQ",
                                          "LOCAL_IR",
                                          "LOCAL_ER",
                                          "NC_REQ",
                                          "ADV_STOP_COMPLETE",
                                          "SCAN_STOP_COMPLETE",
                                          "SET_STATIC_RAND_ADDR",
                                          "UPDATE_CONN_PARAMS",
                                          "SET_PKT_LENGTH_COMPLETE",
                                          "SET_LOCAL_PRIVACY_COMPLETE",
                                          "REMOVE_BOND_DEV_COMPLETE",
                                          "CLEAR_BOND_DEV_COMPLETE",
                                          "GET_BOND_DEV_COMPLETE",
                                          "READ_RSSI_COMPLETE",
                                          "UPDATE_WHITELIST_COMPLETE"};
static const char *bt_gap_evt_names[] = {"DISC_RES",
                                         "DISC_STATE_CHANGED",
                                         "RMT_SRVCS",
                                         "RMT_SRVC_REC",
                                         "AUTH_CMPL",
                                         "PIN_REQ",
                                         "CFM_REQ",
                                         "KEY_NOTIF",
                                         "KEY_REQ",
                                         "READ_RSSI_DELTA"};

const char *ble_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(ble_gap_evt_names)) {
        return "UNKNOWN";
    }
    return ble_gap_evt_names[event];
}

const char *bt_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(bt_gap_evt_names)) {
        return "UNKNOWN";
    }
    return bt_gap_evt_names[event];
}

#if CONFIG_BT_BLE_ENABLED
const char *esp_ble_key_type_str(esp_ble_key_type_t key_type) {
    const char *key_str = NULL;
    switch (key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;
    }
    return key_str;
}
#endif /* CONFIG_BT_BLE_ENABLED */

#if CONFIG_BT_HID_DEVICE_ENABLED
/*
 * BT GAP
 * */

static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        ESP_LOGV(TAG,
                 "BT GAP DISC_STATE %s",
                 (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) ? "START" : "STOP");
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            SEND_BT_CB();
        }
        break;
    }
    case ESP_BT_GAP_DISC_RES_EVT: {
        break;
    }
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
        ESP_LOGI(TAG, "BT GAP READ_REMOTE_NAME status:%d", param->read_rmt_name.stat);
        ESP_LOGI(TAG, "BT GAP READ_REMOTE_NAME name:%s", param->read_rmt_name.rmt_name);
        break;
    }
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "BT GAP KEY_NOTIF passkey:%lu", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "BT GAP MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        esp_bt_gap_read_remote_name(param->mode_chg.bda);
        break;
    default:
        ESP_LOGV(TAG, "BT GAP EVENT %s", bt_gap_evt_str(event));
        break;
    }
}

static esp_err_t init_bt_gap(void) {
    esp_err_t ret;
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    /*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    if ((ret = esp_bt_gap_register_callback(bt_gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %d", ret);
        return ret;
    }

    // Allow BT devices to connect back to us
    if ((ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_set_scan_mode failed: %d", ret);
        return ret;
    }
    return ret;
}

#endif

#if CONFIG_BT_BLE_ENABLED
/*
 * BLE GAP
 * */

esp_bd_addr_t ble_peer_address;

esp_bd_addr_t *get_ble_peer_address(void) { return &ble_peer_address; }

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    /*
     * SCAN
     * */
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN_PARAM_SET_COMPLETE");
        SEND_BLE_CB();
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        ESP_LOGI(TAG, "BLE GAP EVENT SCAN_RESULT");
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN CANCELED");
        break;
    }

    /*
     * ADVERTISEMENT
     * */
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP ADV_DATA_SET_COMPLETE");
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE GAP ADV_START_COMPLETE");
        // zero out the peer address
        memset(ble_peer_address, 0, ESP_BD_ADDR_LEN);
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE GAP ADV_STOP_COMPLETE");
        break;

    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        ESP_LOGI(TAG, "BLE GAP ADV_TERMINATED");
        break;

    /*
     * CONNECTION
     * */
    case ESP_GAP_BLE_GET_DEV_NAME_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP GET_DEV_NAME_COMPLETE");
        // print the name
        ESP_LOGI(TAG, "BLE GAP DEVICE NAME: %s", param->get_dev_name_cmpl.name);
        break;

    /*
     * BOND
     * */
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP REMOVE_BOND_DEV_COMPLETE");
        // log the bond that was removed
        esp_log_buffer_hex(TAG, param->remove_bond_dev_cmpl.bd_addr, ESP_BD_ADDR_LEN);
        break;

    case ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP CLEAR_BOND_DEV_COMPLETE");
        break;

    case ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP GET_BOND_DEV_COMPLETE");
        break;

    /*
     * AUTHENTICATION
     * */
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "BLE GAP AUTH ERROR: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(TAG, "BLE GAP AUTH SUCCESS");
            // log the address of the peer device
            esp_log_buffer_hex(TAG, param->ble_security.auth_cmpl.bd_addr, ESP_BD_ADDR_LEN);
            // save the address of the peer device
            memcpy(ble_peer_address, param->ble_security.auth_cmpl.bd_addr, ESP_BD_ADDR_LEN);
        }
        break;

    case ESP_GAP_BLE_KEY_EVT: // shows the ble key info share with peer device to the user.
        ESP_LOGI(TAG,
                 "BLE GAP KEY type = %s",
                 esp_ble_key_type_str(param->ble_security.ble_key.key_type));
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: // ESP_IO_CAP_OUT
        // The app will receive this evt when the IO has Output capability and the peer device IO
        // has Input capability. Show the passkey number to the user to input it in the peer device.
        ESP_LOGI(TAG, "BLE GAP PASSKEY_NOTIF passkey:%lu", param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT: // ESP_IO_CAP_IO
        // The app will receive this event when the IO has DisplayYesNO capability and the peer
        // device IO also has DisplayYesNo capability. show the passkey number to the user to
        // confirm it with the number displayed by peer device.
        ESP_LOGI(TAG, "BLE GAP NC_REQ passkey:%lu", param->ble_security.key_notif.passkey);
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT: // ESP_IO_CAP_IN
        // The app will receive this evt when the IO has Input capability and the peer device IO has
        // Output capability. See the passkey number on the peer device and send it back.
        ESP_LOGI(TAG, "BLE GAP PASSKEY_REQ");
        // esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 1234);
        break;

    case ESP_GAP_BLE_OOB_REQ_EVT:
        // OOB request event
        ESP_LOGI(TAG, "BLE GAP OOB_REQ");
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, TK, sizeof(TK));
        break;

    case ESP_GAP_BLE_SC_OOB_REQ_EVT:
        // secure connection oob request event
        ESP_LOGI(TAG, "BLE GAP SC_OOB_REQ");
        esp_err_t err = esp_ble_sc_oob_req_reply(param->ble_security.ble_req.bd_addr, ble_oob_sec_data.oob_c, ble_oob_sec_data.oob_r);
        break;

    case ESP_GAP_BLE_SC_CR_LOC_OOB_EVT:
        // secure connection create oob data complete event
        ESP_LOGI(TAG, "BLE GAP SC_CR_LOC_OOB");
        // retrieve and store the local oob data
        memcpy(ble_oob_sec_data.oob_c, param->ble_security.oob_data.oob_c, ESP_BT_OCTET16_LEN);
        memcpy(ble_oob_sec_data.oob_r, param->ble_security.oob_data.oob_r, ESP_BT_OCTET16_LEN);
        // save that we've received the local oob data
        created_ble_oob_data = true;
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "BLE GAP SEC_REQ");
        // Send the positive(true) security response to the peer device to accept the security
        // request. If not accept the security request, should send the security response with
        // negative(false) accept value.
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE GAP PHY_UPDATE_COMPLETE");
        break;

    default:
        ESP_LOGV(TAG, "BLE GAP EVENT %s", ble_gap_evt_str(event));
        break;
    }
}

static esp_err_t init_ble_gap(void) {
    esp_err_t ret;

    if ((ret = esp_ble_gap_register_callback(ble_gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", ret);
        return ret;
    }
    return ret;
}

esp_err_t esp_hid_ble_gap_adv_init(uint16_t appearance,
                                   const char *device_name,
                                   const char *manufacturer_name) {

    esp_err_t ret;

    const uint8_t hidd_service_uuid128[] = {
        0xfb,
        0x34,
        0x9b,
        0x5f,
        0x80,
        0x00,
        0x00,
        0x80,
        0x00,
        0x10,
        0x00,
        0x00,
        0x12,
        0x18,
        0x00,
        0x00,
    };

    #if 1
    // config adv data
    esp_ble_adv_data_t ble_adv_config = {
        .set_scan_rsp = false,
        .include_txpower = true,
        .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x000C, // slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(hidd_service_uuid128),
        .p_service_uuid = (uint8_t *)hidd_service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // was 0x06
    };
    // config scan response data
    esp_ble_adv_data_t ble_scan_rsp_config = {
        .set_scan_rsp = true,
        .include_name = true,
        .appearance = appearance,
        .manufacturer_len = sizeof(manufacturer_name),
        .p_manufacturer_data = manufacturer_name,
    };
    #else
    uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,  // Flags
        0x02, 0x0a, 0x09,  // Tx Power Level
        0x03, 0x03, 0x12, 0x18,  // 16-bit Service Class UUIDs
        0x05, 0x12, 0x06, 0x00, 0x0c, 0x00,  //Periphral Connection Interval Range
        0x02, 0x11, 0x03,  // SM OOB Flags
    };

    static uint8_t raw_scan_rsp_data[] = {
        0x03, 0x19, 0xc4, 0x03,  // Appearance
        0x1a, 0x08, 0x4e, 0x46, 0x43, 0x20, 0x58, 0x62, 0x6f, 0x78, 0x20, 0x45, 0x6c, 0x69,
        0x74, 0x65, 0x20, 0x57, 0x69, 0x72, 0x65, 0x6c, 0x65, 0x73, 0x73, 0x20, 0x43,  // Shortened Local Name
    };
    #endif

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_IO;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    uint8_t spec_auth = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;

    // esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, (void*)&PASSKEY, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &spec_auth, sizeof(uint8_t));

    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t key_size = 16; //the key size should be 7~16 bytes

    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1);

    esp_ble_gap_set_device_name(device_name);

    #if 1
    esp_ble_gap_config_adv_data(&ble_adv_config);
    esp_ble_gap_config_adv_data(&ble_scan_rsp_config);
    #else
    esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
    #endif

    return ESP_OK;
}

esp_err_t esp_hid_ble_gap_adv_start(void) {
    static esp_ble_adv_params_t hidd_adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x30,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    return esp_ble_gap_start_advertising(&hidd_adv_params);
}
#endif /* CONFIG_BT_BLE_ENABLED */

/*
 * CONTROLLER INIT
 * */

static esp_err_t init_low_level(uint8_t mode) {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = mode;
#endif
#if CONFIG_BT_HID_DEVICE_ENABLED
    if (mode & ESP_BT_MODE_CLASSIC_BT) {
        bt_cfg.bt_max_acl_conn = 3;
        bt_cfg.bt_max_sync_conn = 3;
    } else
#endif
    {
        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret) {
            ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %d", ret);
            return ret;
        }
    }
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
        return ret;
    }

    ret = esp_bt_controller_enable(mode);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", ret);
        return ret;
    }
#if CONFIG_BT_HID_DEVICE_ENABLED
    if (mode & ESP_BT_MODE_CLASSIC_BT) {
        ret = init_bt_gap();
        if (ret) {
            return ret;
        }
    }
#endif
#if CONFIG_BT_BLE_ENABLED
    if (mode & ESP_BT_MODE_BLE) {
        ret = init_ble_gap();
        if (ret) {
            return ret;
        }
    }
#endif /* CONFIG_BT_BLE_ENABLED */
    return ret;
}

esp_err_t esp_hid_gap_init(uint8_t mode) {
    esp_err_t ret;
    if (!mode || mode > ESP_BT_MODE_BTDM) {
        ESP_LOGE(TAG, "Invalid mode given!");
        return ESP_FAIL;
    }

    if (bt_hidh_cb_semaphore != NULL) {
        ESP_LOGE(TAG, "Already initialised");
        return ESP_FAIL;
    }

    bt_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (bt_hidh_cb_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
        return ESP_FAIL;
    }

    ble_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (ble_hidh_cb_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
        vSemaphoreDelete(bt_hidh_cb_semaphore);
        bt_hidh_cb_semaphore = NULL;
        return ESP_FAIL;
    }

    ret = init_low_level(mode);
    if (ret != ESP_OK) {
        vSemaphoreDelete(bt_hidh_cb_semaphore);
        bt_hidh_cb_semaphore = NULL;
        vSemaphoreDelete(ble_hidh_cb_semaphore);
        ble_hidh_cb_semaphore = NULL;
        return ret;
    }

    return ESP_OK;
}
