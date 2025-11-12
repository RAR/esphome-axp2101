#include "aw9523.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace aw9523 {

static const char *TAG = "aw9523";

const uint8_t AW9523_REG_INPUT_P0 = 0x00;   // RO, states of GPIO input pins
const uint8_t AW9523_REG_INPUT_P1 = 0x01;
const uint8_t AW9523_REG_OUTPUT_P0 = 0x02;
const uint8_t AW9523_REG_OUTPUT_P1 = 0x03;

const uint8_t AW9523_REG_CFG_P0 = 0x04;  // GPIO direction 0: output (default) 1: input
const uint8_t AW9523_REG_CFG_P1 = 0x05;
const uint8_t AW9523_REG_INT_ENBL_P0 = 0x06;
const uint8_t AW9523_REG_INT_ENBL_P1 = 0x07;

const uint8_t AW9523_REG_CHIPID = 0x10;
const uint8_t AW9523_REG_GCR = 0x11;  // max current IMAX (global control register)

const uint8_t AW9523_REG_LED_P0 = 0x12;  // 0: LED 1: GPIO
const uint8_t AW9523_REG_LED_P1 = 0x13;

// PWM registers for LED dimming
// Port 0: P0_0-P0_7 = 0x20-0x27
// Port 1: P1_0-P1_7 = 0x28-0x2F
const uint8_t AW9523_REG_DIM_P0_BASE = 0x20;  // Base address for Port 0 PWM
const uint8_t AW9523_REG_DIM_P1_BASE = 0x28;  // Base address for Port 1 PWM

const uint8_t AW9523_REG_SOFTRESET = 0x7F;

void AW9523Component::setup() {
  uint8_t chip_id = this->reg(AW9523_REG_CHIPID).get();
  if (chip_id != 0x23) {
    ESP_LOGE(TAG, "Can't find AW9523 chip. Read chip ID: 0x%02X", chip_id);
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "AW9523 chip found with ID: 0x%02X", chip_id);

  // Configure GCR (Global Control Register)
  // Bit 4: 0=Push-pull output, 1=Open-drain output  
  // M5Stack CoreS3 uses 0x10 (bit 4 set) for Port 0 push-pull mode
  this->reg(AW9523_REG_GCR) = 0x10;
  
  // set all pins to GPIO mode initially (bit=1 means GPIO, bit=0 means LED)
  this->reg(AW9523_REG_LED_P0) = 0xff;
  this->reg(AW9523_REG_LED_P1) = 0xff;

  // Keep P1_0 in GPIO mode for now (bit=1)
  // This allows OUTPUT register control which seems more reliable
  
  // Set pin directions based on M5Stack CoreS3 configuration
  // CONFIG_P0: 0x18 = 0b00011000 (pins 3,4 are inputs, rest are outputs)
  // CONFIG_P1: 0x0C = 0b00001100 (pins 2,3 are inputs, rest are outputs)
  this->reg(AW9523_REG_CFG_P0) = 0x18;
  this->reg(AW9523_REG_CFG_P1) = 0x0C;

  // Set initial OUTPUT values based on M5Stack CoreS3 configuration
  // Port 0: bits 0,1,2 high = 0x07
  // Port 1: bits 0,1,7 high = 0x83 (includes backlight on P1_0)
  this->reg(AW9523_REG_OUTPUT_P0) = 0x07;
  this->reg(AW9523_REG_OUTPUT_P1) = 0x83;
  
  ESP_LOGCONFIG(TAG, "M5Stack CoreS3 outputs initialized (P0=0x07, P1=0x83)");
  
  // Log register states for debugging
  uint8_t gcr = this->reg(AW9523_REG_GCR).get();
  uint8_t led_p1 = this->reg(AW9523_REG_LED_P1).get();
  uint8_t cfg_p1 = this->reg(AW9523_REG_CFG_P1).get();
  uint8_t out_p0 = this->reg(AW9523_REG_OUTPUT_P0).get();
  uint8_t out_p1 = this->reg(AW9523_REG_OUTPUT_P1).get();
  
  ESP_LOGCONFIG(TAG, "AW9523 configuration:");
  ESP_LOGCONFIG(TAG, "  GCR: 0x%02X, LED_P1: 0x%02X, CFG_P1: 0x%02X", gcr, led_p1, cfg_p1);
  ESP_LOGCONFIG(TAG, "  OUT_P0: 0x%02X, OUT_P1: 0x%02X", out_p0, out_p1);
}

void AW9523Component::loop() {
  // Periodically check and restore P1_7 (BOOST_EN) if it's been turned off
  // This is critical for M5Stack CoreS3 backlight power
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check > 1000) {  // Check every second
    last_check = now;
    
    uint8_t out_p1 = this->reg(AW9523_REG_OUTPUT_P1).get();
    if ((out_p1 & 0x80) == 0) {  // Check if bit 7 (P1_7 BOOST_EN) is LOW
      ESP_LOGW(TAG, "P1_7 (BOOST_EN) was LOW (0x%02X), restoring to HIGH", out_p1);
      out_p1 |= 0x80;  // Set bit 7 HIGH
      this->reg(AW9523_REG_OUTPUT_P1) = out_p1;
    }
  }
}

void AW9523Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Pin Settings:");
  ESP_LOGCONFIG(TAG, "    Port 0:");
  uint8_t led_reg = this->reg(AW9523_REG_LED_P0).get();
  uint8_t gpio_reg = this->reg(AW9523_REG_CFG_P0).get();
  for (int pin = 0; pin < 8; pin++) {
    const char *io = (gpio_reg & (1 << pin)) ? "I" : "O";
    ESP_LOGCONFIG(TAG, "      Pin %d: %s", pin, (led_reg & (1 << pin)) ? io : "L");
  }
  ESP_LOGCONFIG(TAG, "    Port 1:");
  led_reg = this->reg(AW9523_REG_LED_P1).get();
  gpio_reg = this->reg(AW9523_REG_CFG_P1).get();
  for (int pin = 0; pin < 8; pin++) {
    const char *io = (gpio_reg & (1 << pin)) ? "I" : "O";
    ESP_LOGCONFIG(TAG, "      Pin %d: %s", pin, (led_reg & (1 << pin)) ? io : "L");
  }
}

void AW9523Component::set_pin_mode(AW9523Port port, uint8_t pin, AW9523PinMode mode) {
  if (pin > 7)
    return;

  ESP_LOGD(TAG, "set_pin_mode called: Port %d, Pin %d, Mode %d", port, pin, static_cast<int>(mode));

  if (mode == AW9523PinMode::GPIO_OUT) {
    // Set pin as GPIO mode (bit=1)
    const uint8_t led_reg = port == AW9523Port::PORT_0 ? AW9523_REG_LED_P0 : AW9523_REG_LED_P1;
    this->reg(led_reg) |= (1 << pin);
    // Set pin as output (bit=0)
    const uint8_t cfg_reg = port == AW9523Port::PORT_0 ? AW9523_REG_CFG_P0 : AW9523_REG_CFG_P1;
    this->reg(cfg_reg) &= ~(1 << pin);
    
    ESP_LOGD(TAG, "  LED_REG: 0x%02X, CFG_REG: 0x%02X", this->reg(led_reg).get(), this->reg(cfg_reg).get());
  } else if (mode == AW9523PinMode::GPIO_IN) {
    // Set pin as GPIO mode (bit=1)
    const uint8_t led_reg = port == AW9523Port::PORT_0 ? AW9523_REG_LED_P0 : AW9523_REG_LED_P1;
    this->reg(led_reg) |= (1 << pin);
    // Set pin as input (bit=1)
    const uint8_t cfg_reg = port == AW9523Port::PORT_0 ? AW9523_REG_CFG_P0 : AW9523_REG_CFG_P1;
    this->reg(cfg_reg) |= (1 << pin);
  } else {  // LED mode
    // Set pin as LED mode (bit=0)
    const uint8_t led_reg = port == AW9523Port::PORT_0 ? AW9523_REG_LED_P0 : AW9523_REG_LED_P1;
    this->reg(led_reg) &= ~(1 << pin);
  }
}

void software_reset() {
  // AW9523_REG_SOFTRESET
  // Write 00H to generate a reset pulse
}

void AW9523Component::set_pin_value(AW9523Port port, uint8_t pin, bool bit_value) {
  if (this->is_failed())
    return;

  // Check if pin is in LED mode by reading the LED mode register
  uint8_t led_mode_reg = port == AW9523Port::PORT_0 ? AW9523_REG_LED_P0 : AW9523_REG_LED_P1;
  uint8_t led_mode = this->reg(led_mode_reg).get();
  bool is_led_mode = !(led_mode & (1 << pin));  // bit=0 means LED mode
  
  ESP_LOGD(TAG, "Setting Port %d Pin %d to %s (mode: %s)", 
           port, pin, bit_value ? "HIGH" : "LOW", is_led_mode ? "LED" : "GPIO");
  
  if (is_led_mode && port == AW9523Port::PORT_1 && pin == 0) {
    // For P1_0 in LED mode, control via PWM register at 0x28
    this->reg(AW9523_REG_DIM_P1_BASE) = bit_value ? 0xFF : 0x00;
    ESP_LOGD(TAG, "Set PWM register 0x%02X to %d", AW9523_REG_DIM_P1_BASE, bit_value ? 0xFF : 0x00);
  } else {
    // For GPIO mode, control via OUTPUT register
    uint8_t value = (1 << pin);
    uint8_t port_reg = port == AW9523Port::PORT_0 ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1;
    
    if (bit_value) {
      this->reg(port_reg) |= value;
    } else {
      this->reg(port_reg) &= ~value;
    }
    
    // Read back and log the register value
    uint8_t reg_value = this->reg(port_reg).get();
    ESP_LOGD(TAG, "Register 0x%02X now reads: 0x%02X", port_reg, reg_value);
  }
}

bool AW9523Component::read_pin_value(AW9523Port port, uint8_t pin) {
  if (this->is_failed())
    return false;

  uint8_t bit_mask = (1 << pin);
  uint8_t port_reg = port == AW9523Port::PORT_0 ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1;

  return (this->reg(port_reg).get() & bit_mask) > 0;
}

void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  AW9523PinMode mode = AW9523PinMode::GPIO_IN;
  if (flags == gpio::Flags::FLAG_INPUT) {
    mode = AW9523PinMode::GPIO_IN;
  } else if (flags == gpio::Flags::FLAG_OUTPUT) {
    mode = AW9523PinMode::GPIO_OUT;
  }
  this->parent_->set_pin_mode(this->port_, this->pin_, mode);
}

void AW9523GPIOPin::digital_write(bool value) {
  this->parent_->set_pin_value(this->port_, this->pin_, value != this->inverted_);
}

bool AW9523GPIOPin::digital_read() {
  return this->parent_->read_pin_value(this->port_, this->pin_) != this->inverted_;
}

}  // namespace aw9523
}  // namespace esphome
