#include "axp2101.h"
#include "esphome/core/log.h"
#include "esphome/components/number/number.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";
static bool pmu_flag = false;

void IRAM_ATTR axp2101_isr_handler(void* arg) {
    pmu_flag = true;
}

// Helper methods for register access
bool AXP2101Component::writeRegister(uint8_t reg, uint8_t value) {
    return this->write_byte(reg, value);
}

bool AXP2101Component::readRegister(uint8_t reg, uint8_t *value) {
    return this->read_byte(reg, value);
}

bool AXP2101Component::readRegisterMulti(uint8_t reg, uint8_t *data, size_t len) {
    return this->read_bytes(reg, data, len);
}

bool AXP2101Component::setBits(uint8_t reg, uint8_t mask) {
    uint8_t value;
    if (!readRegister(reg, &value)) return false;
    value |= mask;
    return writeRegister(reg, value);
}

bool AXP2101Component::clearBits(uint8_t reg, uint8_t mask) {
    uint8_t value;
    if (!readRegister(reg, &value)) return false;
    value &= ~mask;
    return writeRegister(reg, value);
}

uint8_t AXP2101Component::getChipID() {
    uint8_t chip_id = 0;
    readRegister(AXP2101_CHIP_ID, &chip_id);
    return chip_id;
}

// DC-DC control methods
void AXP2101Component::setDCVoltage(uint8_t channel, uint16_t voltage_mv) {
    uint8_t reg = AXP2101_DC_VOL0 + channel;
    uint8_t value = 0;
    
    // DC1: 1500-3400mV, 100mV/step
    // DC2/3/4: Different ranges - simplified implementation
    if (channel == 0) { // DC1
        if (voltage_mv >= 1500 && voltage_mv <= 3400) {
            value = (voltage_mv - 1500) / 100;
        }
    } else if (channel <= 3) { // DC2-4
        if (voltage_mv >= 500 && voltage_mv <= 1200) {
            value = (voltage_mv - 500) / 10;
        } else if (voltage_mv >= 1220 && voltage_mv <= 1540) {
            value = 71 + (voltage_mv - 1220) / 20;
        }
    } else if (channel == 4) { // DC5
        if (voltage_mv >= 1400 && voltage_mv <= 3700) {
            value = (voltage_mv - 1400) / 100;
        } else if (voltage_mv == 1200) {
            value = 0;
        }
    }
    
    writeRegister(reg, value);
}

void AXP2101Component::enableDC(uint8_t channel, bool enable) {
    uint8_t mask = 1 << channel;
    if (enable) {
        setBits(AXP2101_DC_ONOFF_DVM_CTRL, mask);
    } else {
        clearBits(AXP2101_DC_ONOFF_DVM_CTRL, mask);
    }
}

// LDO control methods
void AXP2101Component::setALDOVoltage(uint8_t channel, uint16_t voltage_mv) {
    if (channel > 3) return;
    uint8_t reg = AXP2101_ALDO_VOL0 + channel;
    
    // ALDO: 500-3500mV, 100mV/step
    if (voltage_mv >= 500 && voltage_mv <= 3500) {
        uint8_t value = (voltage_mv - 500) / 100;
        writeRegister(reg, value);
    }
}

void AXP2101Component::setBLDOVoltage(uint8_t channel, uint16_t voltage_mv) {
    if (channel > 1) return;
    uint8_t reg = AXP2101_BLDO_VOL0 + channel;
    
    // BLDO: 500-3500mV, 100mV/step
    if (voltage_mv >= 500 && voltage_mv <= 3500) {
        uint8_t value = (voltage_mv - 500) / 100;
        writeRegister(reg, value);
    }
}

void AXP2101Component::enableALDO(uint8_t channel, bool enable) {
    if (channel > 3) return;
    uint8_t mask = 1 << channel;
    if (enable) {
        setBits(AXP2101_LDO_ONOFF_CTRL0, mask);
    } else {
        clearBits(AXP2101_LDO_ONOFF_CTRL0, mask);
    }
}

void AXP2101Component::enableBLDO(uint8_t channel, bool enable) {
    if (channel > 1) return;
    uint8_t mask = 1 << (channel + 4);  // BLDO1/2 are bits 4-5
    if (enable) {
        setBits(AXP2101_LDO_ONOFF_CTRL0, mask);
    } else {
        clearBits(AXP2101_LDO_ONOFF_CTRL0, mask);
    }
}

// Battery and charging methods
uint16_t AXP2101Component::getBatteryVoltage() {
    uint8_t data[2];
    if (!readRegisterMulti(AXP2101_VBAT_H, data, 2)) {
        return 0;
    }
    // 14-bit value, LSB = 1.1mV
    uint16_t raw = ((uint16_t)data[0] << 6) | (data[1] & 0x3F);
    return (uint16_t)((float)raw * 1.1f);
}

uint8_t AXP2101Component::getBatteryPercent() {
    uint8_t percent = 0;
    if (!readRegister(AXP2101_BAT_PERCENT, &percent)) {
        return 0;
    }
    return (percent & 0x7F);  // Lower 7 bits
}

bool AXP2101Component::isCharging() {
    uint8_t status = 0;
    if (!readRegister(AXP2101_COMM_STAT0, &status)) {
        return false;
    }
    return (status & 0x40) != 0;  // Bit 6: charging status
}

bool AXP2101Component::isBatteryConnected() {
    uint8_t status = 0;
    if (!readRegister(AXP2101_COMM_STAT0, &status)) {
        return false;
    }
    return (status & 0x08) != 0;  // Bit 3: battery present
}

bool AXP2101Component::isVBUSGood() {
    uint8_t status = 0;
    if (!readRegister(AXP2101_COMM_STAT0, &status)) {
        ESP_LOGW(TAG, "Failed to read status register for VBUS check");
        return false;
    }
    ESP_LOGD(TAG, "COMM_STAT0 register: 0x%02X", status);
    // Check multiple status bits for VBUS presence
    // Bit 5: VBUS good status
    // Bit 7: VBUS present
    bool vbus_good = (status & 0x20) != 0;
    bool vbus_present = (status & 0x80) != 0;
    ESP_LOGD(TAG, "VBUS good bit: %d, VBUS present bit: %d", vbus_good, vbus_present);
    return vbus_good || vbus_present;
}

void AXP2101Component::setChargingLedMode(uint8_t mode) {
    if (mode > 4) return;  // Valid modes: 0-4
    
    uint8_t val = 0;
    if (!readRegister(AXP2101_CHGLED_SET_CTRL, &val)) {
        ESP_LOGW(TAG, "Failed to read charging LED control register");
        return;
    }
    
    // Clear bits 4-5 (mode) and bit 2 (manual control enable)
    val &= 0xC8;  // Keep bits 7,6,3,1,0 unchanged
    
    if (mode == 4) {
        // Mode 4: Automatic control by charger (bit 2 = 0)
        val |= 0x01;  // Set bit 0 to enable LED function
    } else {
        // Modes 0-3: Manual control (bit 2 = 1)
        val |= 0x05;  // Set bits 2,0 for manual mode and LED enable
        val |= (mode << 4);  // Set mode in bits 4-5
    }
    
    writeRegister(AXP2101_CHGLED_SET_CTRL, val);
    ESP_LOGD(TAG, "Set charging LED mode to %d (reg value: 0x%02X)", mode, val);
}

uint8_t AXP2101Component::getChargingLedMode() {
    uint8_t val = 0;
    if (!readRegister(AXP2101_CHGLED_SET_CTRL, &val)) {
        return 0;
    }
    
    // Check if manual mode (bit 2 = 1) or automatic (bit 2 = 0)
    if (val & 0x04) {
        // Manual mode: return mode from bits 4-5
        return (val >> 4) & 0x03;
    } else {
        // Automatic mode
        return 4;
    }
}

uint16_t AXP2101Component::getVBUSVoltage() {
    uint8_t data[2];
    if (!readRegisterMulti(AXP2101_VBUS_H, data, 2)) {
        ESP_LOGW(TAG, "Failed to read VBUS voltage registers");
        return 0;
    }
    // 14-bit value, LSB = 1.1mV (same as battery voltage)
    uint16_t raw = ((uint16_t)data[0] << 6) | (data[1] & 0x3F);
    ESP_LOGD(TAG, "VBUS raw ADC: %d (H:0x%02X L:0x%02X)", raw, data[0], data[1]);
    return (uint16_t)((float)raw * 1.1f);
}

float AXP2101Component::getTemperature() {
    uint8_t data[2];
    if (!readRegisterMulti(AXP2101_TS_H, data, 2)) {
        return 0.0f;
    }
    // 14-bit value: High 6 bits in data[0], Low 8 bits in data[1]
    // XPowersLib uses readRegisterH6L8: (data[0] << 8) | data[1], masked to 14 bits
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x3FFF;  // Mask to 14 bits
    // Formula from XPowersLib: T(°C) = 22.0 + (7274 - raw) / 20.0
    // This gives internal die temperature in °C
    float temp = 22.0f + (7274.0f - (float)raw) / 20.0f;
    return temp;
}

void AXP2101Component::initPowerOutputs() {
    ESP_LOGI(TAG, "Initializing power outputs...");
    
    // IMPORTANT: Don't change voltages for critical system rails that are already on
    // Only configure rails that are safe to modify
    
    // Read current power states to avoid disrupting critical rails
    uint8_t dc_state, ldo_state0, ldo_state1;
    readRegister(AXP2101_DC_ONOFF_DVM_CTRL, &dc_state);
    readRegister(AXP2101_LDO_ONOFF_CTRL0, &ldo_state0);
    readRegister(AXP2101_LDO_ONOFF_CTRL1, &ldo_state1);
    
    ESP_LOGD(TAG, "Current power states - DC: 0x%02X, LDO0: 0x%02X, LDO1: 0x%02X", dc_state, ldo_state0, ldo_state1);
    
    // Only set BLDO1 voltage for backlight control (safe to modify)
    ESP_LOGD(TAG, "Setting BLDO1 for backlight...");
    setBLDOVoltage(0, 3300);  // BLDO1: Backlight
    
    // Ensure BLDO1 is enabled for backlight
    bool bldo1_enabled = (ldo_state0 & (1 << 4));
    ESP_LOGD(TAG, "BLDO1 status: %s (bit 4 = %d)", bldo1_enabled ? "ENABLED" : "DISABLED", bldo1_enabled);
    
    if (!bldo1_enabled) {
        ESP_LOGD(TAG, "Enabling BLDO1...");
        enableBLDO(0, true);
    } else {
        ESP_LOGD(TAG, "BLDO1 already enabled, verifying voltage is set...");
    }
    
    // Read back to verify
    uint8_t bldo1_voltage;
    readRegister(AXP2101_BLDO_VOL0, &bldo1_voltage);
    ESP_LOGD(TAG, "BLDO1 voltage register: 0x%02X (%d mV)", bldo1_voltage, (bldo1_voltage * 100) + 500);
    
    ESP_LOGI(TAG, "Power outputs initialized (conservative mode)");
}

void AXP2101Component::initCharger() {
    ESP_LOGI(TAG, "Initializing charger...");
    
    // Read current ADC state
    uint8_t adc_state;
    readRegister(AXP2101_ADC_ENABLE, &adc_state);
    ESP_LOGD(TAG, "Current ADC state: 0x%02X", adc_state);
    
    // Enable ADC for battery, VBUS, and temperature monitoring
    // Bit 7: VBAT, Bit 1: VBUS, Bit 0: TS (die temperature)
    // Note: Die temperature and TS pin are different - bit 0 is for internal die temp
    setBits(AXP2101_ADC_ENABLE, 0x83);  // Enable VBAT (0x80), VBUS (0x02), and die temp (0x01)
    
    // Verify ADC settings
    readRegister(AXP2101_ADC_ENABLE, &adc_state);
    ESP_LOGI(TAG, "ADC enabled - state: 0x%02X (VBAT:%d VBUS:%d TEMP:%d)", 
             adc_state, 
             (adc_state & 0x80) ? 1 : 0,
             (adc_state & 0x02) ? 1 : 0, 
             (adc_state & 0x01) ? 1 : 0);
    
    // Note: Not modifying VBUS limits, charge current, etc. as they may already be
    // configured by the bootloader/hardware. This is safer for initial setup.
    
    ESP_LOGI(TAG, "Charger initialized (basic ADC only)");
}

void AXP2101Component::initInterrupts() {
    ESP_LOGI(TAG, "Initializing interrupts...");
    
    // Configure power button timing first (register 0x23)
    // Bits 7-6: Power-on press time
    // Bits 5-4: IRQ level time  
    // Bits 3-2: Power-off long press time
    // Bits 1-0: Power-on long press time
    // Set to: Power-on=128ms, IRQ=1s, Power-off=4s, Long=1s
    uint8_t pok_config = (0 << 6) | (0 << 4) | (0 << 2) | (0 << 0); // All shortest timings
    writeRegister(AXP2101_POK_SET, pok_config);
    ESP_LOGI(TAG, "Button timing configured: POK_SET=0x%02X", pok_config);
    
    // Skip GPIO interrupt setup for now - can be added later if needed
    ESP_LOGD(TAG, "Skipping GPIO IRQ pin setup (not critical for basic operation)");
    
    // Clear all interrupt flags
    writeRegister(AXP2101_IRQ_STATUS0, 0xFF);
    writeRegister(AXP2101_IRQ_STATUS1, 0xFF);
    writeRegister(AXP2101_IRQ_STATUS2, 0xFF);
    
    // Disable all interrupts first
    writeRegister(AXP2101_IRQ_EN0, 0x00);
    writeRegister(AXP2101_IRQ_EN1, 0x00);
    writeRegister(AXP2101_IRQ_EN2, 0x00);
    
    // Enable low battery warning interrupts if any binary sensors are configured
    uint8_t irq_en0 = 0x00;
    if (low_battery_level1_bsensor_ != nullptr) {
        irq_en0 |= AXP2101_WARNING_LEVEL1_IRQ;
        ESP_LOGD(TAG, "Enabled low battery warning level 1 interrupt");
    }
    if (low_battery_level2_bsensor_ != nullptr) {
        irq_en0 |= AXP2101_WARNING_LEVEL2_IRQ;
        ESP_LOGD(TAG, "Enabled low battery warning level 2 interrupt");
    }
    
    if (irq_en0 != 0x00) {
        writeRegister(AXP2101_IRQ_EN0, irq_en0);
        ESP_LOGI(TAG, "Low battery warning interrupts enabled: 0x%02X", irq_en0);
    }
    
    // Enable power button interrupts if any binary sensors are configured
    uint8_t irq_en1 = 0x00;
    if (pkey_short_bsensor_ != nullptr) {
        irq_en1 |= AXP2101_PKEY_SHORT_IRQ;
        ESP_LOGD(TAG, "Enabled PKEY short press interrupt");
    }
    if (pkey_long_bsensor_ != nullptr) {
        irq_en1 |= AXP2101_PKEY_LONG_IRQ;
        ESP_LOGD(TAG, "Enabled PKEY long press interrupt");
    }
    if (pkey_positive_bsensor_ != nullptr) {
        irq_en1 |= AXP2101_PKEY_POSITIVE_IRQ;
        ESP_LOGD(TAG, "Enabled PKEY positive edge interrupt");
    }
    if (pkey_negative_bsensor_ != nullptr) {
        irq_en1 |= AXP2101_PKEY_NEGATIVE_IRQ;
        ESP_LOGD(TAG, "Enabled PKEY negative edge interrupt");
    }
    
    if (irq_en1 != 0x00) {
        writeRegister(AXP2101_IRQ_EN1, irq_en1);
        ESP_LOGI(TAG, "Power button interrupts enabled: 0x%02X", irq_en1);
        
        // Verify the register was written correctly
        uint8_t verify = 0;
        readRegister(AXP2101_IRQ_EN1, &verify);
        ESP_LOGI(TAG, "IRQ_EN1 verification readback: 0x%02X", verify);
    } else {
        ESP_LOGD(TAG, "No power button sensors configured, interrupts disabled");
    }
    
    ESP_LOGI(TAG, "Interrupts initialized (polling mode)");
}

void AXP2101Component::setup() {
    ESP_LOGCONFIG(TAG, "Setting up AXP2101...");
    ESP_LOGCONFIG(TAG, "I2C Address: 0x%02X", this->address_);
    
    // Small delay to allow I2C bus to stabilize
    delay_microseconds_safe(100000);  // 100ms
    
    // Test basic I2C communication by trying to read multiple registers
    ESP_LOGD(TAG, "Testing I2C communication...");
    uint8_t test_val;
    
    // Try reading chip ID first (register 0x03)
    bool comm_ok = readRegister(AXP2101_CHIP_ID, &test_val);
    if (comm_ok) {
        ESP_LOGD(TAG, "Successfully read Chip ID register: 0x%02X", test_val);
    } else {
        ESP_LOGW(TAG, "Failed to read Chip ID register");
    }
    
    // Try reading status register (register 0x01)
    if (readRegister(AXP2101_COMM_STAT0, &test_val)) {
        ESP_LOGD(TAG, "Successfully read Status register: 0x%02X", test_val);
        comm_ok = true;
    } else {
        ESP_LOGW(TAG, "Failed to read Status register");
    }
    
    // Try reading config register (register 0x00)
    if (readRegister(AXP2101_COMM_CFG, &test_val)) {
        ESP_LOGD(TAG, "Successfully read Config register: 0x%02X", test_val);
        comm_ok = true;
    } else {
        ESP_LOGW(TAG, "Failed to read Config register");
    }
    
    if (!comm_ok) {
        ESP_LOGE(TAG, "Failed to communicate with AXP2101!");
        ESP_LOGE(TAG, "Please check:");
        ESP_LOGE(TAG, "  - I2C bus is correctly initialized (SDA: GPIO21, SCL: GPIO22)");
        ESP_LOGE(TAG, "  - I2C address is correct (0x34 for AXP2101)");
        ESP_LOGE(TAG, "  - Power is supplied to the AXP2101");
        ESP_LOGE(TAG, "  - Pull-up resistors are present on I2C lines");
        this->mark_failed();
        return;
    }
    
    // Read and verify chip ID
    uint8_t chip_id = getChipID();
    ESP_LOGCONFIG(TAG, "Chip ID: 0x%02X", chip_id);
    
    // AXP2101 chip ID should be 0x47, but some variants may differ
    // 0x4A is a valid AXP2101 variant chip ID
    if (chip_id == 0x00 || chip_id == 0xFF) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X - possible I2C communication error", chip_id);
        this->mark_failed();
        return;
    } else if (chip_id == 0x47 || chip_id == 0x4A) {
        ESP_LOGI(TAG, "Valid AXP2101 chip ID detected: 0x%02X", chip_id);
    } else {
        ESP_LOGW(TAG, "Unexpected chip ID! Expected 0x47 or 0x4A, got 0x%02X - continuing anyway", chip_id);
    }
    
    // Initialize subsystems
    ESP_LOGI(TAG, "Initializing AXP2101 subsystems...");
    initPowerOutputs();
    initCharger();
    initInterrupts();
    
    // Publish initial brightness to number component if set
    if (this->brightness_number_ != nullptr) {
        this->brightness_number_->publish_state(this->brightness_ * 100.0f);
    }
    
    // Initialize state-based binary sensors to OFF
    // (Don't initialize momentary sensors like short/long press - they auto-clear)
    if (this->low_battery_level1_bsensor_ != nullptr) {
        this->low_battery_level1_bsensor_->publish_state(false);
    }
    if (this->low_battery_level2_bsensor_ != nullptr) {
        this->low_battery_level2_bsensor_->publish_state(false);
    }
    // Note: pkey_positive sensor appears inverted - starts TRUE (button not pressed)
    // Will go FALSE when button pressed, TRUE when released
    if (this->pkey_positive_bsensor_ != nullptr) {
        this->pkey_positive_bsensor_->publish_state(true);
    }
    if (this->pkey_negative_bsensor_ != nullptr) {
        this->pkey_negative_bsensor_->publish_state(false);
    }
    
    ESP_LOGCONFIG(TAG, "AXP2101 setup complete");
}

void AXP2101Component::dump_config() {
    ESP_LOGCONFIG(TAG, "AXP2101:");
    LOG_I2C_DEVICE(this);
    
    if (this->is_failed()) {
        ESP_LOGE(TAG, "Communication with AXP2101 failed!");
        return;
    }
    
    ESP_LOGCONFIG(TAG, "  Chip ID: 0x%02X", getChipID());
    LOG_SENSOR("  ", "Battery Voltage", this->batteryvoltage_sensor_);
    LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
    LOG_BINARY_SENSOR("  ", "Battery Charging", this->batterycharging_bsensor_);
    LOG_SENSOR("  ", "VBUS Voltage", this->vbusvoltage_sensor_);
    LOG_BINARY_SENSOR("  ", "VBUS Connected", this->vbusconnected_bsensor_);
    LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
    ESP_LOGCONFIG(TAG, "  Brightness: %.0f%%", this->brightness_ * 100.0f);
}

float AXP2101Component::get_setup_priority() const { 
    return setup_priority::DATA; 
}

void AXP2101Component::update() {
    // Always read interrupt status registers to check for events
    uint8_t irq[3];
    readRegister(AXP2101_IRQ_STATUS0, &irq[0]);
    readRegister(AXP2101_IRQ_STATUS1, &irq[1]);
    readRegister(AXP2101_IRQ_STATUS2, &irq[2]);
    
    // Check for low battery warning events in IRQ_STATUS0
    if (irq[0] != 0x00) {
        ESP_LOGD(TAG, "IRQ_STATUS0: 0x%02X", irq[0]);
        
        // Warning level 1 (higher threshold, first warning)
        if ((irq[0] & AXP2101_WARNING_LEVEL1_IRQ) && low_battery_level1_bsensor_ != nullptr) {
            ESP_LOGW(TAG, "Low battery warning level 1!");
            low_battery_level1_bsensor_->publish_state(true);
        }
        
        // Warning level 2 (lower threshold, critical warning)
        if ((irq[0] & AXP2101_WARNING_LEVEL2_IRQ) && low_battery_level2_bsensor_ != nullptr) {
            ESP_LOGW(TAG, "Low battery warning level 2 (critical)!");
            low_battery_level2_bsensor_->publish_state(true);
        }
        
        // Clear the interrupt flags we just processed
        writeRegister(AXP2101_IRQ_STATUS0, irq[0]);
    }
    
    // Check for power button events in IRQ_STATUS1
    if (irq[1] != 0x00) {
        ESP_LOGD(TAG, "IRQ_STATUS1: 0x%02X", irq[1]);
        
        // Short press event
        if ((irq[1] & AXP2101_PKEY_SHORT_IRQ) && pkey_short_bsensor_ != nullptr) {
            ESP_LOGD(TAG, "Power button short press");
            pkey_short_bsensor_->publish_state(true);
            delay_microseconds_safe(50000);  // 50ms delay to ensure event is registered
            pkey_short_bsensor_->publish_state(false);
        }
        
        // Long press event
        if ((irq[1] & AXP2101_PKEY_LONG_IRQ) && pkey_long_bsensor_ != nullptr) {
            ESP_LOGD(TAG, "Power button long press");
            pkey_long_bsensor_->publish_state(true);
            delay_microseconds_safe(50000);  // 50ms delay to ensure event is registered
            pkey_long_bsensor_->publish_state(false);
        }
        
        // Positive edge (button pressed down)
        if ((irq[1] & AXP2101_PKEY_POSITIVE_IRQ) && pkey_positive_bsensor_ != nullptr) {
            ESP_LOGD(TAG, "Power button pressed (positive edge)");
            pkey_positive_bsensor_->publish_state(false);  // Inverted: goes OFF when pressed
        }
        
        // Negative edge (button released)
        if ((irq[1] & AXP2101_PKEY_NEGATIVE_IRQ)) {
            ESP_LOGD(TAG, "Power button released (negative edge)");
            // Set the positive edge sensor (button no longer held)
            if (pkey_positive_bsensor_ != nullptr) {
                pkey_positive_bsensor_->publish_state(true);  // Inverted: goes ON when released
            }
            // Trigger the negative edge sensor momentarily
            if (pkey_negative_bsensor_ != nullptr) {
                pkey_negative_bsensor_->publish_state(true);
                delay_microseconds_safe(50000);  // 50ms delay
                pkey_negative_bsensor_->publish_state(false);
            }
        }
        
        // Clear the interrupt flags we just processed
        writeRegister(AXP2101_IRQ_STATUS1, irq[1]);
    }
    
    // Clear other interrupt flags if present
    if (irq[2] != 0x00) {
        writeRegister(AXP2101_IRQ_STATUS2, irq[2]);
    }
    
    // Update battery voltage
    if (this->batteryvoltage_sensor_ != nullptr) {
        uint16_t vbat_mv = getBatteryVoltage();
        float vbat_v = vbat_mv / 1000.0f;
        ESP_LOGD(TAG, "Battery Voltage: %.3fV (%dmV)", vbat_v, vbat_mv);
        this->batteryvoltage_sensor_->publish_state(vbat_v);
    }
    
    // Update battery level
    if (this->batterylevel_sensor_ != nullptr) {
        float battery_level;
        
        if (isBatteryConnected()) {
            battery_level = getBatteryPercent();
        } else {
            // Fallback calculation if battery gauge not available
            uint16_t vbat_mv = getBatteryVoltage();
            float vbat_v = vbat_mv / 1000.0f;
            battery_level = (vbat_v - 3.0f) / (4.1f - 3.0f) * 100.0f;
        }
        
        if (battery_level > 100.0f) battery_level = 100.0f;
        if (battery_level < 0.0f) battery_level = 0.0f;
        
        ESP_LOGD(TAG, "Battery Level: %.0f%%", battery_level);
        this->batterylevel_sensor_->publish_state(battery_level);
        
        // Clear low battery warnings if battery level improves
        // Typically level 1 = ~15%, level 2 = ~5%
        if (battery_level > 15.0f && low_battery_level1_bsensor_ != nullptr) {
            low_battery_level1_bsensor_->publish_state(false);
        }
        if (battery_level > 5.0f && low_battery_level2_bsensor_ != nullptr) {
            low_battery_level2_bsensor_->publish_state(false);
        }
    }
    
    // Update charging status
    if (this->batterycharging_bsensor_ != nullptr) {
        bool charging = isCharging();
        ESP_LOGD(TAG, "Battery Charging: %s", charging ? "Yes" : "No");
        this->batterycharging_bsensor_->publish_state(charging);
    }
    
    // Update VBUS voltage
    if (this->vbusvoltage_sensor_ != nullptr) {
        uint16_t vbus_mv = getVBUSVoltage();
        float vbus_v = vbus_mv / 1000.0f;
        ESP_LOGD(TAG, "VBUS Voltage: %.3fV (%dmV)", vbus_v, vbus_mv);
        this->vbusvoltage_sensor_->publish_state(vbus_v);
    }
    
    // Update VBUS connected status
    if (this->vbusconnected_bsensor_ != nullptr) {
        bool vbus_good = isVBUSGood();
        ESP_LOGD(TAG, "VBUS Connected: %s", vbus_good ? "Yes" : "No");
        this->vbusconnected_bsensor_->publish_state(vbus_good);
    }
    
    // Update temperature
    if (this->temperature_sensor_ != nullptr) {
        float temp = getTemperature();
        ESP_LOGD(TAG, "Temperature: %.1f°C", temp);
        this->temperature_sensor_->publish_state(temp);
    }
    
    // Update brightness
    UpdateBrightness();
}

void AXP2101Component::UpdateBrightness() {
    if (brightness_ == curr_brightness_) return;
    
    uint16_t new_voltage_mv = 500;  // Minimum voltage (off)
    
    if (brightness_ > 0.0f) {
        // Map brightness 0.0-1.0 to voltage 2500-3300mV
        const float min_voltage = 2500.0f;
        const float max_voltage = 3300.0f;
        const float voltage_step = 100.0f;
        
        float target_mv = min_voltage + (brightness_ * (max_voltage - min_voltage));
        new_voltage_mv = (uint16_t)(std::ceil(target_mv / voltage_step) * voltage_step);
        
        ESP_LOGD(TAG, "Setting backlight: %.0f%% -> %dmV", brightness_ * 100.0f, new_voltage_mv);
    } else {
        ESP_LOGD(TAG, "Turning backlight off");
    }
    
    curr_brightness_ = brightness_;
    
    // BLDO1 controls backlight on M5Core2
    setBLDOVoltage(0, new_voltage_mv);
}

void AXP2101Component::SetSleep() {
    // Prepare for sleep
    writeRegister(AXP2101_SLEEP_CFG, 0x08);
}

void AXP2101Component::DeepSleep(uint64_t time_in_us) {
    SetSleep();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0);
    
    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    
    esp_deep_sleep_start();
}

void AXP2101Component::LightSleep(uint64_t time_in_us) {
    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    
    esp_light_sleep_start();
}

std::string AXP2101Component::GetStartupReason() {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    
    if (reset_reason == ESP_RST_DEEPSLEEP) {
        esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
        switch (wake_reason) {
            case ESP_SLEEP_WAKEUP_EXT0: return "ESP_SLEEP_WAKEUP_EXT0";
            case ESP_SLEEP_WAKEUP_EXT1: return "ESP_SLEEP_WAKEUP_EXT1";
            case ESP_SLEEP_WAKEUP_TIMER: return "ESP_SLEEP_WAKEUP_TIMER";
            case ESP_SLEEP_WAKEUP_TOUCHPAD: return "ESP_SLEEP_WAKEUP_TOUCHPAD";
            case ESP_SLEEP_WAKEUP_ULP: return "ESP_SLEEP_WAKEUP_ULP";
            case ESP_SLEEP_WAKEUP_GPIO: return "ESP_SLEEP_WAKEUP_GPIO";
            case ESP_SLEEP_WAKEUP_UART: return "ESP_SLEEP_WAKEUP_UART";
            default: return "WAKEUP_UNKNOWN_REASON";
        }
    }
    
    switch (reset_reason) {
        case ESP_RST_UNKNOWN: return "ESP_RST_UNKNOWN";
        case ESP_RST_POWERON: return "ESP_RST_POWERON";
        case ESP_RST_SW: return "ESP_RST_SW";
        case ESP_RST_PANIC: return "ESP_RST_PANIC";
        case ESP_RST_INT_WDT: return "ESP_RST_INT_WDT";
        case ESP_RST_TASK_WDT: return "ESP_RST_TASK_WDT";
        case ESP_RST_WDT: return "ESP_RST_WDT";
        case ESP_RST_BROWNOUT: return "ESP_RST_BROWNOUT";
        case ESP_RST_SDIO: return "ESP_RST_SDIO";
        default: return "RESET_UNKNOWN_REASON";
    }
}

}  // namespace axp2101
}  // namespace esphome
