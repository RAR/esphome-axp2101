#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

namespace esphome {
namespace axp2101 {

enum AXP2101Model {
  AXP2101_M5CORE2,
};

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define CURRENT_100MA  (0b0000)
#define CURRENT_190MA  (0b0001)
#define CURRENT_280MA  (0b0010)
#define CURRENT_360MA  (0b0011)
#define CURRENT_450MA  (0b0100)
#define CURRENT_550MA  (0b0101)
#define CURRENT_630MA  (0b0110)
#define CURRENT_700MA  (0b0111)

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
public:
  void set_batteryvoltage_sensor(sensor::Sensor *batteryvoltage_sensor) { batteryvoltage_sensor_ = batteryvoltage_sensor; }
  void set_batterylevel_sensor(sensor::Sensor *batterylevel_sensor) { batterylevel_sensor_ = batterylevel_sensor; }
  void set_brightness(float brightness) { brightness_ = brightness; }

  void set_batterycharging_bsensor(binary_sensor::BinarySensor *batterycharging_bsensor) { batterycharging_bsensor_ = batterycharging_bsensor; }
  void set_model(AXP2101Model model) { this->model_ = model; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

private:
  int readRegister(uint8_t reg);
  int writeRegister(uint8_t reg, uint8_t *buf, uint8_t length);
  uint16_t getBLDO1Voltage(void);
  bool setBLDO1Voltage(uint16_t millivolt);


protected:
  sensor::Sensor *batteryvoltage_sensor_;
  sensor::Sensor *batterylevel_sensor_;
  binary_sensor::BinarySensor *batterycharging_bsensor_;

  float brightness_{.50f};
  float curr_brightness_{-1.0f};
  AXP2101Model model_;
};

}
}

#endif
