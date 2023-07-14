#pragma once

#include "format.hpp"

#if CONFIG_BT_BLE_ENABLED

#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"

#include "esp_hid_gap.h"
#include "esp_hidd.h"

void remove_all_bonded_devices();
esp_hidd_dev_t *init_ble_hid(int VID,
                             int PID,
                             const char *manufacturer_name,
                             const char *device_name,
                             const std::string &serial_number);

#endif
