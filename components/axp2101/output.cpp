#include "output.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp2101 {

static const char *const TAG = "axp2101.output";

void AXP2101Output::setup() {
  // Enable DLDO1 and set initial brightness to 100%
  this->write_state(1.0f);
}

void AXP2101Output::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP2101 DLDO1 Backlight Output");
}

void AXP2101Output::write_state(float state) {
  // state is 0.0 to 1.0
  
  if (state > 0) {
    // Enable DLDO1 (register 0x90 bit 7)
    uint8_t current_val;
    if (this->parent_->read_byte(0x90, &current_val)) {
      this->parent_->write_byte(0x90, current_val | 0x80);
    }
    
    // Set DLDO1 voltage (register 0x99)
    // Brightness range: 0.5V-3.4V in 100mV steps
    // Formula from M5GFX: brightness = ((brightness + 641) >> 5)
    uint8_t brightness_byte = (uint8_t)(state * 255.0f);
    uint8_t voltage_level = (uint8_t)(((uint16_t)brightness_byte + 641) >> 5);
    
    this->parent_->write_byte(0x99, voltage_level);
    
    ESP_LOGD(TAG, "Set backlight brightness: %.1f%% (voltage_level: %d)", state * 100.0f, voltage_level);
  } else {
    // Disable DLDO1
    uint8_t current_val;
    if (this->parent_->read_byte(0x90, &current_val)) {
      this->parent_->write_byte(0x90, current_val & ~0x80);
    }
    
    ESP_LOGD(TAG, "Backlight disabled");
  }
}

}  // namespace axp2101
}  // namespace esphome
