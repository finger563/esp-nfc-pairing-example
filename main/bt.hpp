#pragma once

#include <atomic>
#include <chrono>
#include <thread>

extern std::atomic<bool> connected_bt;

#include "esp_bt_device.h"
#include "esp_bt_main.h"

#include "esp_hid_gap.h"
#include "esp_hidd.h"

#if CONFIG_BT_HID_DEVICE_ENABLED
esp_hidd_dev_t *init_bt_hid(int VID,
                            int PID,
                            const char *manufacturer_name,
                            const char *device_name,
                            const std::string &serial_number);
#endif
