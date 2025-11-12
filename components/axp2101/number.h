#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "axp2101.h"

namespace esphome {
namespace axp2101 {

class AXP2101Component;

class AXP2101Number : public number::Number, public Component {
 public:
  void set_parent(AXP2101Component *parent) { this->parent_ = parent; }
  void dump_config() override;

 protected:
  void control(float value) override;
  AXP2101Component *parent_;
};

}  // namespace axp2101
}  // namespace esphome
