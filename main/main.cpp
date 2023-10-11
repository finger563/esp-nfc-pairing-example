#include <chrono>
#include <thread>

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <esp_random.h>
#include <driver/i2c.h>
#include <nvs_flash.h>

#include "esp_hid_gap.h"
#include "ble.hpp"
#include "bt.hpp"

#include "logger.hpp"
#include "ndef.hpp"
#include "st25dv.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto I2C_NUM = I2C_NUM_1;
#if CONFIG_IDF_TARGET_ESP32S3
static constexpr auto I2C_SCL_IO = GPIO_NUM_40; // Qwiic SCL on QtPy ESP32S3
static constexpr auto I2C_SDA_IO = GPIO_NUM_41; // Qwiic SDA on QtPy ESP32S3
#else
static constexpr auto I2C_SCL_IO = GPIO_NUM_19; // Qwiic SCL on QtPy ESP32 PICO
static constexpr auto I2C_SDA_IO = GPIO_NUM_22; // Qwiic SDA on QtPy ESP32 PICO
#endif
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = 10;

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  espp::Logger logger({.tag = "NFC BLE OOB", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  // Initialize NVS.
  auto ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // make the I2C that we'll use to communicate
  i2c_config_t i2c_cfg;
  logger.info("initializing i2c driver...");
  memset(&i2c_cfg, 0, sizeof(i2c_cfg));
  i2c_cfg.sda_io_num = I2C_SDA_IO;
  i2c_cfg.scl_io_num = I2C_SCL_IO;
  i2c_cfg.mode = I2C_MODE_MASTER;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
  auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
  if (err != ESP_OK)
    logger.error("config i2c driver failed");
  err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK)
    logger.error("install i2c driver failed");
  // make some lambda functions we'll use to read/write to the st25dv
  auto i2c_write = [](uint8_t addr, uint8_t *data, uint8_t length) {
    i2c_master_write_to_device(I2C_NUM, addr, data, length, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  auto i2c_write_read = [](uint8_t addr, uint16_t reg_addr, uint8_t *data, uint8_t length) {
    uint8_t reg[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF)};
    i2c_master_write_read_device(I2C_NUM, addr, reg, 2, data, length,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  // now make the st25dv which decodes the data
  espp::St25dv st25dv({
      .write = i2c_write,
      .read = i2c_write_read,
      .log_level = espp::Logger::Verbosity::DEBUG
    });

  // initialize bluedroid stack
#if CONFIG_BT_HID_DEVICE_ENABLED || CONFIG_BT_BLE_ENABLED
    err = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(err);
#endif

  // don't want a lot of logs (and all the logs can cause the ble_hid task stack
  // to overflow)
  esp_log_level_set("HID_DEV_BLE", ESP_LOG_NONE);
  esp_log_level_set("ESP_HID_GAP", ESP_LOG_INFO);

  // get the mac address of the radio
  const uint8_t* point = esp_bt_dev_get_address();
  uint64_t radio_mac_addr = 0;
  if (point == nullptr) {
    logger.error("get bt mac address failed");
    return;
  } else {
    // convert the 6 byte mac address to a 48 bit integer
    for (int i = 0; i < 6; i++) {
      radio_mac_addr |= (uint64_t)point[5-i] << (i * 8);
    }
  }
  logger.info("radio mac addr: {:#x}", radio_mac_addr);

  // set the bluetooth name
  std::string device_name = "NFC Xbox Elite Wireless Controller";
  esp_bt_dev_set_device_name(device_name.c_str());

  uint32_t random_number = esp_random();
  std::string serial_number = fmt::format("{:010d}", random_number);
  logger.info("Generated serial number: {}", serial_number);

  uint16_t VID = 0x045E; // Microsoft
  uint16_t PID = 0x02FD; // Xbox Elite Wireless Controller
  std::string manufacturer_name = "Microsoft Corporation";

#if CONFIG_BT_BLE_ENABLED
  [[maybe_unused]] auto ble_hid_dev = init_ble_hid(VID,
                                                   PID,
                                                   manufacturer_name.c_str(),
                                                   device_name.c_str(),
                                                   serial_number);
#endif

#if CONFIG_BT_HID_DEVICE_ENABLED
  [[maybe_unused]] auto bt_hid_dev = init_bt_hid(VID,
                                                 PID,
                                                 manufacturer_name.c_str(),
                                                 device_name.c_str(),
                                                 serial_number);
#endif

  //////////////BLE SECURITY SETUP//////////////////////

  // create BLE OOB pairing record
  logger.info("Waiting for OOB data to be created...");
  while (!has_created_ble_oob_sec_data()) {
    ESP_ERROR_CHECK(esp_ble_create_sc_oob_data());
    std::this_thread::sleep_for(100ms);
    #if CONFIG_IDF_TARGET_ESP32
    // on the ESP32 we cannot create the OOB data for some reason...
    logger.warn("Cannot create OOB data on ESP32, skipping OOB data creation");
    break;
    #endif
  }

  // now copy the bytes from oob_sec_data into the correct variables
  std::string confirmation_value{""};
  std::string randomizer_value{""};

  #if !CONFIG_IDF_TARGET_ESP32
  confirmation_value.resize(16, 0);
  randomizer_value.resize(16, 0);
  auto oob_data_ptr = get_ble_oob_sec_data_ptr();
  memcpy(confirmation_value.data(), oob_data_ptr->oob_c, 16);
  memcpy(randomizer_value.data(), oob_data_ptr->oob_r, 16);

  // for printing purposes
  std::vector<uint8_t> oob_r;
  oob_r.resize(16, 0);
  std::vector<uint8_t> oob_c;
  oob_c.resize(16, 0);
  memcpy(oob_r.data(), randomizer_value.data(), randomizer_value.size());
  memcpy(oob_c.data(), confirmation_value.data(), confirmation_value.size());

  logger.debug("confirmation value: {::02x}", oob_c);
  logger.debug("randomizer value: {::02x}", oob_r);
  #endif

  // get the temporary key from the esp_ble_sec_t struct
  // it's within esp_ble_sec_t under the oob_data field
  uint8_t tk_data[16] = {
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
    0x00, 0x00, 0x00, 0x11,
  };
  std::string_view tk{reinterpret_cast<char*>(tk_data), 16};

  /////////////NDEF RECORDS/////////////////////

  // make a payload id for the oob record
  int payload_id = '0';

  // Bluetooth device class for gamepad, see
  // https://www.ampedrftech.com/datasheets/cod_definition.pdf
  // and is represented as 3 octets, with octet 1 being LSB
  // High 6 bits of octet 1 is Minor Device Class
  // Low 5 bits of octet 2 is Major Device Class
  // Octet 3 + high 3 bits of octet 2 is Major Service Class
  // 0x002508: Gamepad
  // Major Service Class: 0x002000: Limited Discoverable Mode
  // Major Device Class:  0x000500: Peripheral
  // Minor Device Class:  0x000008: Gamepad
  uint32_t device_class = 0x002508;
  auto ble_role = espp::Ndef::BleRole::PERIPHERAL_ONLY;
  auto ble_appearance = espp::Ndef::BtAppearance::GAMEPAD;

  std::vector<espp::Ndef> records;
  records.emplace_back(espp::Ndef::make_handover_select(payload_id));
  // records.emplace_back(espp::Ndef::make_oob_pairing(radio_mac_addr, device_class, device_name));
  records.emplace_back(espp::Ndef::make_le_oob_pairing(radio_mac_addr, ble_role, device_name, ble_appearance,
                                                       randomizer_value, confirmation_value, tk));

  // set the id of the ble oob record
  records[1].set_id(payload_id);

  st25dv.set_records(records);

  /////////////FIELD MONITORING/////////////////////

  // Make a task that will run in the background and print the interrupt status
  // when it changes
  espp::Task task({
      .name = "St25dv Field Change",
      .callback = [&](auto &m, auto &cv) -> bool {
        auto it_sts = st25dv.get_interrupt_status();
        static auto last_it_sts = it_sts;
        if (it_sts != last_it_sts) {
          logger.info("[{:.3f}] IT STS: {:02x}", elapsed(), it_sts);
        }
        last_it_sts = it_sts;
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 10ms);
        // we don't want to stop the task, so return false
        return false;
      },
      .stack_size_bytes = 5 * 1024,
    });
  task.start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
