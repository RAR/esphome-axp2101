#include "number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp2101 {

static const char *const TAG = "axp2101.number";

void AXP2101Number::setup() {
  float value;
  if (!this->restore_value) {
    value = this->initial_value;
  } else {
    this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
    if (!this->pref_.load(&value)) {
      value = this->initial_value;
    }
  }
  this->publish_state(value);
  // Apply the initial brightness
  float brightness = value / 100.0f;
  this->parent_->set_brightness(brightness);
}

void AXP2101Number::dump_config() {
  LOG_NUMBER("", "AXP2101 Backlight", this);
}

void AXP2101Number::control(float value) {
  if (this->restore_value) {
    this->pref_.save(&value);
  }
  // Convert percentage (0-100) to float (0.0-1.0)
  float brightness = value / 100.0f;
  this->parent_->set_brightness(brightness);
  this->publish_state(value);
}

void AXP2101ChargingLedNumber::dump_config() {
  LOG_NUMBER("", "AXP2101 Charging LED", this);
}

void AXP2101ChargingLedNumber::control(float value) {
  uint8_t mode = static_cast<uint8_t>(value);
  if (mode <= 4) {
    this->parent_->setChargingLedMode(mode);
    this->publish_state(value);
  }
}

}  // namespace axp2101
}  // namespace esphome
