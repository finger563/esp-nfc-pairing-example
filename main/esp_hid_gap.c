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

static SemaphoreHandle_t ble_hidh_cb_semaphore = NULL;
#define WAIT_BLE_CB() xSemaphoreTake(ble_hidh_cb_semaphore, portMAX_DELAY)
#define SEND_BLE_CB() xSemaphoreGive(ble_hidh_cb_semaphore)

#define SIZEOF_ARRAY(a) (sizeof(a) / sizeof(*a))

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

const char *ble_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(ble_gap_evt_names)) {
        return "UNKNOWN";
    }
    return ble_gap_evt_names[event];
}

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
        ESP_LOGW(TAG, "BLE GAP OOB_REQ");
        uint8_t TK[16] = {0x01};
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, TK, sizeof(TK));
        break;

    case ESP_GAP_BLE_SC_OOB_REQ_EVT:
        // secure connection oob request event
        ESP_LOGW(TAG, "BLE GAP SC_OOB_REQ");
        // create confirmation value (128-bit random number)
        uint8_t C[16] = {0x00};
        // create randomizer value, should be 128bit random number
        uint8_t R[16] = {0x00};
        esp_ble_sc_oob_req_reply(param->ble_security.ble_req.bd_addr, C, R);
        break;

    case ESP_GAP_BLE_SC_CR_LOC_OOB_EVT:
        // secure connection create oob data complete event
        ESP_LOGW(TAG, "BLE GAP SC_CR_LOC_OOB");
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "BLE GAP SEC_REQ");
        // Send the positive(true) security response to the peer device to accept the security
        // request. If not accept the security request, should send the security response with
        // negative(false) accept value.
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    default:
        ESP_LOGW(TAG, "BLE GAP EVENT %s - %d", ble_gap_evt_str(event), event);
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

    // config adv data
    esp_ble_adv_data_t ble_adv_config = {
        .set_scan_rsp = false,
        .include_txpower = true,
        .min_interval = 0x0018, // slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0020, // slave connection max interval, Time = max_interval * 1.25 msec
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

    // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_MITM;
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1)) !=
        ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param AUTHEN_REQ_MODE failed: %d", ret);
        return ret;
    }

    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; // device is not capable of input or output, unsecure
    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param IOCAP_MODE failed: %d", ret);
        return ret;
    }

    uint8_t oob_support = ESP_BLE_OOB_ENABLE;
    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t))) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param OOB_SUPPORT failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_device_name(device_name)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_device_name failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_config_adv_data(&ble_adv_config)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP config_adv_data failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_config_adv_data(&ble_scan_rsp_config)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP config_adv_data ble_scan_rsp_config failed: %d", ret);
        return ret;
    }

    return ret;
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

/*
 * CONTROLLER INIT
 * */

static esp_err_t init_low_level() {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = ESP_BT_MODE_BLE;
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

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
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
    ret = init_ble_gap();
    if (ret) {
        return ret;
    }
    return ret;
}

esp_err_t esp_hid_gap_init() {
    esp_err_t ret;
    ble_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (ble_hidh_cb_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
        return ESP_FAIL;
    }

    ret = init_low_level();
    if (ret != ESP_OK) {
        vSemaphoreDelete(ble_hidh_cb_semaphore);
        ble_hidh_cb_semaphore = NULL;
        return ret;
    }

    return ESP_OK;
}
