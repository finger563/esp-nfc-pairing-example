#include <chrono>
#include <thread>

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <driver/i2c.h>
#include <nvs_flash.h>

#include "logger.hpp"
#include "ndef.hpp"
#include "st25dv.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto I2C_NUM = I2C_NUM_1;
static constexpr auto I2C_SCL_IO = GPIO_NUM_19;
static constexpr auto I2C_SDA_IO = GPIO_NUM_22;
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
  logger.info("initializing bluedroid...");
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  err = esp_bt_controller_init(&bt_cfg);
  if (err != ESP_OK)
    logger.error("initialize controller failed");

  err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (err != ESP_OK)
    logger.error("enable controller failed");

  err = esp_bluedroid_init();
  if (err != ESP_OK)
    logger.error("initialize bluedroid failed");

  err = esp_bluedroid_enable();
  if (err != ESP_OK)
    logger.error("enable bluedroid failed");

  // get the mac address of the radio
  const uint8_t* point = esp_bt_dev_get_address();
  uint64_t radio_mac_addr = 0;
  if (point == nullptr) {
    logger.error("get bt mac address failed");
  } else {
    // convert the 6 byte mac address to a 48 bit integer
    for (int i = 0; i < 6; i++) {
      radio_mac_addr |= (uint64_t)point[i] << (i * 8);
    }
  }
  logger.info("radio mac addr: {::#x}", radio_mac_addr);

  // set the bluetooth name
  std::string device_name = "My Xbox Wireless Controller";
  esp_bt_dev_set_device_name(device_name.c_str());

  // create BT OOB pairing record
  uint32_t bt_device_class = 0x000000;      // 24b
  auto bt_oob_record =
    espp::Ndef::make_oob_pairing(radio_mac_addr, bt_device_class, device_name);

  // create BLE OOB pairing record
  auto ble_role = espp::Ndef::BleRole::PERIPHERAL_ONLY;
  auto ble_appearance = espp::Ndef::BtAppearance::GAMEPAD;
  auto ble_oob_record =
    espp::Ndef::make_le_oob_pairing(radio_mac_addr, ble_role, device_name, ble_appearance);

  // set one of the records we made to be the active tag
  st25dv.set_record(ble_oob_record);

  logger.info("ble oob record:  {::#x}", ble_oob_record.payload());

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
      .stack_size_bytes = 4096,
    });
  task.start();

  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
  }
}
