#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "axp2101.h"

namespace esphome {
namespace axp2101 {

class AXP2101Output : public output::FloatOutput, public Component {
 public:
  void set_parent(AXP2101Component *parent) { this->parent_ = parent; }
  void setup() override;
  void dump_config() override;

 protected:
  void write_state(float state) override;
  
  AXP2101Component *parent_;
};

}  // namespace axp2101
}  // namespace esphome
