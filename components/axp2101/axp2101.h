#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

static constexpr int TFT_DISPOFF = 0x28;
static constexpr int TFT_DISPON  = 0x29;

namespace esphome {
namespace axp2101 {

enum AXP2101Model {
  AXP2101_M5CORE2,
};

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {

public:
  void set_brightness(float brightness) { brightness_ = brightness; }
  void set_model(AXP2101Model model) { this->model_ = model; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

private:
  void bitOn(uint8_t reg, uint8_t bit);
  void bitOff(uint8_t reg, uint8_t bit);
  void writeRegister8(uint8_t reg, uint8_t data, uint8_t mask);
  void i2c_write_register8_array(const uint8_t* reg_data_mask);
  void setBrightness(uint8_t brightness);

protected:
  float brightness_{.50f};
  float curr_brightness_{-1.0f};
  AXP2101Model model_;
};

}
}

#endif
