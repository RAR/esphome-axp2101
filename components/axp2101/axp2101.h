#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "driver/gpio.h"

// Forward declaration
namespace esphome {
namespace number {
class Number;
}
}

namespace esphome {
namespace axp2101 {

// AXP2101 Register Definitions
#define AXP2101_I2C_ADDR 0x34

// Power control registers
#define AXP2101_COMM_CFG 0x00
#define AXP2101_COMM_STAT0 0x01
#define AXP2101_COMM_STAT1 0x02
#define AXP2101_CHIP_ID 0x03

// Power output control
#define AXP2101_DC_WORK_MODE 0x80
#define AXP2101_DC_VOL0 0x82
#define AXP2101_DC_VOL1 0x83
#define AXP2101_DC_VOL2 0x84
#define AXP2101_DC_VOL3 0x85
#define AXP2101_DC_VOL4 0x86

// LDO control
#define AXP2101_ALDO_VOL0 0x92
#define AXP2101_ALDO_VOL1 0x93
#define AXP2101_ALDO_VOL2 0x94
#define AXP2101_ALDO_VOL3 0x95
#define AXP2101_BLDO_VOL0 0x96
#define AXP2101_BLDO_VOL1 0x97
#define AXP2101_CPUSLDO_VOL 0x98
#define AXP2101_DLDO_VOL0 0x99
#define AXP2101_DLDO_VOL1 0x9A

// Power enable control
#define AXP2101_LDO_ONOFF_CTRL0 0x90
#define AXP2101_LDO_ONOFF_CTRL1 0x91
#define AXP2101_DC_ONOFF_DVM_CTRL 0x81

// Battery/ADC
#define AXP2101_ADC_ENABLE 0x30
#define AXP2101_VBAT_H 0x34
#define AXP2101_VBAT_L 0x35
#define AXP2101_TS_H 0x36
#define AXP2101_TS_L 0x37
#define AXP2101_VBUS_H 0x38
#define AXP2101_VBUS_L 0x39
#define AXP2101_VSYS_H 0x3A
#define AXP2101_VSYS_L 0x3B

// Charge control
#define AXP2101_CHG_CFG 0x61
#define AXP2101_CHG_ITERM 0x62
#define AXP2101_CHG_IPRE_CC 0x63
#define AXP2101_CHG_VOL 0x64

// IRQ control
#define AXP2101_IRQ_EN0 0x40
#define AXP2101_IRQ_EN1 0x41
#define AXP2101_IRQ_EN2 0x42
#define AXP2101_IRQ_STATUS0 0x48
#define AXP2101_IRQ_STATUS1 0x49
#define AXP2101_IRQ_STATUS2 0x4A

// IRQ bit masks (IRQ_STATUS0 / IRQ_EN0)
#define AXP2101_WARNING_LEVEL1_IRQ  (1 << 6)  // Battery level drops to warning level 1
#define AXP2101_WARNING_LEVEL2_IRQ  (1 << 7)  // Battery level drops to warning level 2

// IRQ bit masks (IRQ_STATUS1 / IRQ_EN1)
#define AXP2101_PKEY_POSITIVE_IRQ   (1 << 0)  // Power button pressed (falling edge)
#define AXP2101_PKEY_NEGATIVE_IRQ   (1 << 1)  // Power button released (rising edge)
#define AXP2101_PKEY_LONG_IRQ       (1 << 2)  // Power button long press
#define AXP2101_PKEY_SHORT_IRQ      (1 << 3)  // Power button short press

// Charging LED control
#define AXP2101_CHGLED_SET_CTRL 0x69

// Battery gauge
#define AXP2101_BAT_PERCENT 0xA4

// Misc
#define AXP2101_VBUS_VOL_SET 0x16
#define AXP2101_VBUS_CUR_SET 0x15
#define AXP2101_VSYS_MIN 0x17
#define AXP2101_POK_SET 0x23
#define AXP2101_SLEEP_CFG 0x27
#define AXP2101_WAKEUP_CFG 0x28
#define AXP2101_WDT_CTRL 0x29

enum AXP2101Model {
  AXP2101_M5CORE2,
  AXP2101_M5CORES3,
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
  void set_vbusvoltage_sensor(sensor::Sensor *vbusvoltage_sensor) { vbusvoltage_sensor_ = vbusvoltage_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_brightness(float brightness) { brightness_ = brightness; UpdateBrightness(); }
  void set_brightness_number(number::Number *brightness_number) { brightness_number_ = brightness_number; }

  void set_batterycharging_bsensor(binary_sensor::BinarySensor *batterycharging_bsensor) { batterycharging_bsensor_ = batterycharging_bsensor; }
  void set_vbusconnected_bsensor(binary_sensor::BinarySensor *vbusconnected_bsensor) { vbusconnected_bsensor_ = vbusconnected_bsensor; }
  void set_pkey_short_bsensor(binary_sensor::BinarySensor *pkey_short_bsensor) { pkey_short_bsensor_ = pkey_short_bsensor; }
  void set_pkey_long_bsensor(binary_sensor::BinarySensor *pkey_long_bsensor) { pkey_long_bsensor_ = pkey_long_bsensor; }
  void set_pkey_positive_bsensor(binary_sensor::BinarySensor *pkey_positive_bsensor) { pkey_positive_bsensor_ = pkey_positive_bsensor; }
  void set_pkey_negative_bsensor(binary_sensor::BinarySensor *pkey_negative_bsensor) { pkey_negative_bsensor_ = pkey_negative_bsensor; }
  void set_low_battery_level1_bsensor(binary_sensor::BinarySensor *low_battery_level1_bsensor) { low_battery_level1_bsensor_ = low_battery_level1_bsensor; }
  void set_low_battery_level2_bsensor(binary_sensor::BinarySensor *low_battery_level2_bsensor) { low_battery_level2_bsensor_ = low_battery_level2_bsensor; }
  void set_model(AXP2101Model model) { this->model_ = model; }
  void set_charging_led_number(number::Number *charging_led_number) { charging_led_number_ = charging_led_number; }

  // Charging LED control
  void setChargingLedMode(uint8_t mode);  // 0=Off, 1=Blink 1Hz, 2=Blink 4Hz, 3=On, 4=Auto
  uint8_t getChargingLedMode();

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

private:
    static std::string GetStartupReason();

protected:
    sensor::Sensor *batteryvoltage_sensor_{nullptr};
    sensor::Sensor *batterylevel_sensor_{nullptr};
    sensor::Sensor *vbusvoltage_sensor_{nullptr};
    sensor::Sensor *temperature_sensor_{nullptr};
    binary_sensor::BinarySensor *batterycharging_bsensor_{nullptr};
    binary_sensor::BinarySensor *vbusconnected_bsensor_{nullptr};
    binary_sensor::BinarySensor *pkey_short_bsensor_{nullptr};
    binary_sensor::BinarySensor *pkey_long_bsensor_{nullptr};
    binary_sensor::BinarySensor *pkey_positive_bsensor_{nullptr};
    binary_sensor::BinarySensor *pkey_negative_bsensor_{nullptr};
    binary_sensor::BinarySensor *low_battery_level1_bsensor_{nullptr};
    binary_sensor::BinarySensor *low_battery_level2_bsensor_{nullptr};
    number::Number *brightness_number_{nullptr};
    number::Number *charging_led_number_{nullptr};

    float brightness_{.50f};
    float curr_brightness_{-1.0f};
    AXP2101Model model_;
    gpio_num_t irq_pin_{GPIO_NUM_21};

    /* 
     * M5Stack Core2 Values
     * DC1: Internal 3.3V
     * DC2: Internal 1.0V
     * DC3: Display power (3.3V)
     * DC4: Internal 1.0V
     * DC5: External 3.3V
     * ALDO1-4: Various LDOs
     * BLDO1: Backlight control
     * BLDO2: Secondary power
     */

    // Helper methods for register access
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readRegisterMulti(uint8_t reg, uint8_t *data, size_t len);
    bool setBits(uint8_t reg, uint8_t mask);
    bool clearBits(uint8_t reg, uint8_t mask);

    // AXP2101 control methods
    void initPowerOutputs();
    void initCharger();
    void initInterrupts();
    uint16_t getBatteryVoltage();  // Returns mV
    uint8_t getBatteryPercent();
    bool isCharging();
    bool isBatteryConnected();
    bool isVBUSGood();
    uint16_t getVBUSVoltage();  // Returns mV
    float getTemperature();  // Returns °C
    uint8_t getChipID();
    
    // DC-DC converter control
    void setDCVoltage(uint8_t channel, uint16_t voltage_mv);
    void enableDC(uint8_t channel, bool enable);
    
    // LDO control
    void setALDOVoltage(uint8_t channel, uint16_t voltage_mv);
    void setBLDOVoltage(uint8_t channel, uint16_t voltage_mv);
    void enableALDO(uint8_t channel, bool enable);
    void enableBLDO(uint8_t channel, bool enable);

    void UpdateBrightness();
    void SetSleep();
    void DeepSleep(uint64_t time_in_us = 0);
    void LightSleep(uint64_t time_in_us = 0);
};

}
}

#endif
