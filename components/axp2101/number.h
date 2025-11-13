#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "axp2101.h"

namespace esphome {
namespace axp2101 {

class AXP2101Component;

class AXP2101Number : public number::Number, public Component {
 public:
  void set_parent(AXP2101Component *parent) { this->parent_ = parent; }
  void set_initial_value(float value) { this->initial_value = value; }
  void set_restore_value(bool restore) { this->restore_value = restore; }
  void setup() override;
  void dump_config() override;

 protected:
  void control(float value) override;
  AXP2101Component *parent_;
  float initial_value{100.0f};
  bool restore_value{true};
  ESPPreferenceObject pref_;
};

class AXP2101ChargingLedNumber : public number::Number, public Component {
 public:
  void set_parent(AXP2101Component *parent) { this->parent_ = parent; }
  void dump_config() override;

 protected:
  void control(float value) override;
  AXP2101Component *parent_;
};

}  // namespace axp2101
}  // namespace esphome
