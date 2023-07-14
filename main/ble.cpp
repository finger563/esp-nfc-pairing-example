#include "ble.hpp"

static const unsigned char hidapiReportMap[] = {
    // 8 bytes input, 8 bytes feature
    0x06,
    0x00,
    0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x0A,
    0x00,
    0x01, // Usage (0x0100)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    0x01, //   Report ID (1)
    0x15,
    0x00, //   Logical Minimum (0)
    0x26,
    0xFF,
    0x00, //   Logical Maximum (255)
    0x75,
    0x08, //   Report Size (8)
    0x95,
    0x08, //   Report Count (8)
    0x09,
    0x01, //   Usage (0x01)
    0x82,
    0x02,
    0x01, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,
          //          No Null Position, Buffered Bytes)
    0x95,
    0x08, //   Report Count (8)
    0x09,
    0x02, //   Usage (0x02)
    0xB2,
    0x02,
    0x01, //   Feature (Data,Var,Abs,No Wrap,Linear, Preferred State,
          //            No Null Position, Non-volatile, Buffered
          //            Bytes)
    0x95,
    0x08, //   Report Count (8)
    0x09,
    0x03, //   Usage (0x03)
    0x91,
    0x02, //   Output (Data,Var,Abs,No Wrap,Linear, Preferred
          //           State,No Null Position, Non-volatile)
    0xC0, // End Collection
};

#if CONFIG_BT_BLE_ENABLED
static esp_hid_raw_report_map_t ble_report_maps[] = {
    {.data = hidapiReportMap, .len = sizeof(hidapiReportMap)},
};

static esp_hid_device_config_t ble_hid_config = {.vendor_id = 0x00,
                                                 .product_id = 0x00,
                                                 .version = 0x0100,
                                                 .device_name = "",
                                                 .manufacturer_name = "",
                                                 .serial_number = "1234567890",
                                                 .report_maps = ble_report_maps,
                                                 .report_maps_len = 1};

void remove_all_bonded_devices(void) {
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list =
        static_cast<esp_ble_bond_dev_t *>(malloc(sizeof(esp_ble_bond_dev_t) * dev_num));
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void
ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = static_cast<esp_hidd_event_data_t *>(event_data);
    static const char *TAG = "HID_DEV_BLE";

    switch (event) {
    case ESP_HIDD_START_EVENT: {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        ESP_LOGI(TAG, "CONNECT");
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGV(TAG,
                 "PROTOCOL MODE[%u]: %s",
                 param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT: {
        ESP_LOGV(TAG,
                 "CONTROL[%u]: %sSUSPEND",
                 param->control.map_index,
                 param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGV(TAG,
                 "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:",
                 param->output.map_index,
                 esp_hid_usage_str(param->output.usage),
                 param->output.report_id,
                 param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGV(TAG,
                 "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:",
                 param->feature.map_index,
                 esp_hid_usage_str(param->feature.usage),
                 param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        ESP_LOGI(TAG,
                 "DISCONNECT: %s",
                 esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev),
                                               param->disconnect.reason));
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}

esp_hidd_dev_t *init_ble_hid(int VID,
                             int PID,
                             const char *manufacturer_name,
                             const char *device_name,
                             const std::string &serial_number) {
    esp_hidd_dev_t *ble_hid_dev;

    // make sure the serial number is set
    ble_hid_config.vendor_id = VID;
    ble_hid_config.product_id = PID;
    ble_hid_config.device_name = device_name;
    ble_hid_config.manufacturer_name = manufacturer_name;
    ble_hid_config.serial_number = serial_number.c_str();

    auto err = esp_hid_ble_gap_adv_init(
        ESP_HID_APPEARANCE_GAMEPAD, ble_hid_config.device_name, ble_hid_config.manufacturer_name);
    ESP_ERROR_CHECK(err);

    if ((err = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        fmt::print(fg(fmt::terminal_color::red), "GATTS register callback failed: {}\n", err);
        return nullptr;
    }
    ESP_ERROR_CHECK(esp_hidd_dev_init(
        &ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &ble_hid_dev));
    return ble_hid_dev;
}

#endif
