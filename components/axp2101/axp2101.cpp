#include "axp2101.h"
#include "esp_sleep.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "driver/i2c.h"
#include "REG/AXP2101Constants.h"

#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

const uint8_t i2c_sda = CONFIG_PMU_SDA;
const uint8_t i2c_scl = CONFIG_PMU_SCL;

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";

void AXP2101Component::setup() {
    ESP_LOGD(TAG, "Getting Backlight Status:");
    float cur_volt = static_cast<float>(this->getBLDO1Voltage());
    ESP_LOGD(TAG, "Curr: %f", cur_volt);
    this->setBLDO1Voltage(3300.00);
    cur_volt = static_cast<float>(this->getBLDO1Voltage());
    ESP_LOGD(TAG, "After: %f", cur_volt);
}

void AXP2101Component::dump_config() {}
float AXP2101Component::get_setup_priority() const { return setup_priority::DATA; }
void AXP2101Component::update() {}

int AXP2101Component::readRegister(uint8_t reg){
    uint8_t val = 0;
    return this->read_bytes(reg, &val, 1);
}

int AXP2101Component::writeRegister(uint8_t reg, uint8_t *buf, uint8_t length){
    uint8_t *write_buffer = (uint8_t *)malloc(sizeof(uint8_t) * 2);
    write_buffer[0] = reg;
    memcpy(write_buffer + 1, buf, 1);
    
    return this->write_register(reg, write_buffer, 1);
}

uint16_t AXP2101Component::getBLDO1Voltage(void) {
    int val = readRegister(XPOWERS_AXP2101_LDO_VOL4_CTRL);
    if (val == -1)return 0;
    val &= 0x1F;
    return val * XPOWERS_AXP2101_BLDO1_VOL_STEPS + XPOWERS_AXP2101_BLDO1_VOL_MIN;
}

bool AXP2101Component::setBLDO1Voltage(uint16_t millivolt){
    if (millivolt % XPOWERS_AXP2101_BLDO1_VOL_STEPS) {
        ESP_LOGD(TAG, "Mistake ! The steps is must %u mV", XPOWERS_AXP2101_BLDO1_VOL_STEPS);
        return false;
    }
    if (millivolt < XPOWERS_AXP2101_BLDO1_VOL_MIN) {
        ESP_LOGD(TAG, "Mistake ! BLDO1 minimum output voltage is  %umV", XPOWERS_AXP2101_BLDO1_VOL_MIN);
        return false;
    } else if (millivolt > XPOWERS_AXP2101_BLDO1_VOL_MAX) {
        ESP_LOGD(TAG, "Mistake ! BLDO1 maximum output voltage is  %umV", XPOWERS_AXP2101_BLDO1_VOL_MAX);
        return false;
    }
    uint8_t val = readRegister(XPOWERS_AXP2101_LDO_VOL4_CTRL);
    if (val == -1)return  false;
    val &= 0xE0;
    val |= (millivolt - XPOWERS_AXP2101_BLDO1_VOL_MIN) / XPOWERS_AXP2101_BLDO1_VOL_STEPS;
    ESP_LOGD(TAG, "Setting To: %f", val);
    return 0 == writeRegister(XPOWERS_AXP2101_LDO_VOL4_CTRL, &val, 1);
}    

}
}
