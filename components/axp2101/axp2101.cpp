#include "axp2101.h"
#include "esphome/core/log.h"
#include <unistd.h>

// for Core2 v1.1 (AXP2101)
// ALDO2 == LCD+TOUCH RST
// ALDO3 == SPK EN
// ALDO4 == TF, TP, LCD PWR
// BLDO1 == LCD BL
// BLDO2 == Boost EN
// DLDO1 == Vibration Motor
static constexpr uint8_t reg_data_axp2101_first[] = {
  0x90, 0x08, 0x7B,   // ALDO4 ON / ALDO3 OFF, DLDO1 OFF
  0x80, 0x05, 0xFF,   // DCDC1 + DCDC3 ON
  0x82, 0x12, 0x00,   // DCDC1 3.3V
  0x84, 0x6A, 0x00,   // DCDC3 3.3V
  0xFF, 0xFF, 0xFF,
};

static constexpr uint8_t reg_data_axp2101_reset[] = {
  0x90, 0x00, 0xFD,   // ALDO2 OFF
  0xFF, 0xFF, 0xFF,
};

static constexpr uint8_t reg_data_axp2101_second[] = {
  0x90, 0x02, 0xFF,   // ALDO2 ON
  0xFF, 0xFF, 0xFF,
};

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";

void AXP2101Component::setup() {
    ESP_LOGD(TAG, "Running Setup:");
    ESP_LOGD(TAG, " General Setup...");
    this->i2c_write_register8_array(reg_data_axp2101_first);
    ESP_LOGD(TAG, " Display Reset...");
    this->i2c_write_register8_array(reg_data_axp2101_reset);
    ESP_LOGD(TAG, " Enabling Display...");
    this->i2c_write_register8_array(reg_data_axp2101_second);
    this->setBrightness(200);
}

void AXP2101Component::dump_config() {}
float AXP2101Component::get_setup_priority() const { return setup_priority::DATA; }
void AXP2101Component::update() {
    ESP_LOGD(TAG, "Running Update...");
}

void AXP2101Component::setBrightness(uint8_t brightness) {
  // BLDO1
  ESP_LOGD(TAG, "Setting Brightness to %i", brightness);
  if (brightness) {
    brightness = ((brightness + 641) >> 5);
    this->bitOn(0x90, 0x10); // BLDO1 enable reg, bit
  }
  else {
    this->bitOff(0x90, 0x10); // BLDO1 disable reg, bit
  }
  usleep(3);
  this->writeRegister8(0x96, brightness, 0);
}

void AXP2101Component::bitOn(uint8_t reg, uint8_t bit) {
  return this->writeRegister8(reg, bit, ~0); // reg, data, mask
}
void AXP2101Component::bitOff(uint8_t reg, uint8_t bit) {
  return this->writeRegister8(reg, 0, ~bit); // reg, data, mask
}

void AXP2101Component::writeRegister8(uint8_t reg, uint8_t data, uint8_t mask) {
  auto res = this->write_bytes(reg, &data, 1);
}

void AXP2101Component::i2c_write_register8_array(const uint8_t* reg_data_mask) {
  while (reg_data_mask[0] != 0xFF || reg_data_mask[1] != 0xFF || reg_data_mask[2] != 0xFF) {
    ESP_LOGD(TAG, "    Setting reg: %X, data: %X, mask: %X", reg_data_mask[0], reg_data_mask[1], reg_data_mask[2]);
    this->writeRegister8(reg_data_mask[0], reg_data_mask[1], reg_data_mask[2]);
    reg_data_mask += 3;
    usleep(3);
  }
}

}
}
