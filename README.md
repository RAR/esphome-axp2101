# ESPHome AXP2101 Component

This custom component provides **ESP-IDF native** support for the AXP2101 power management IC used in M5Stack devices. This is a complete rewrite using direct I2C register access instead of Arduino libraries, making it compatible with ESP-IDF framework projects.

**Supported Devices:**
- [M5Stack Core2 V1.1](https://docs.m5stack.com/en/core/Core2%20v1.1) - Uses AXP2101 with AXP192 compatibility
- [M5Stack CoreS3](https://docs.m5stack.com/en/core/CoreS3) - Uses AXP2101

**Key Features:**
- ESP-IDF native implementation (no Arduino framework required)
- Direct I2C register access (no XPowersLib dependency)
- Battery monitoring (voltage, level, charging status)
- Model-specific backlight control (BLDO1 for Core2, DLDO1 for CoreS3)
- Conservative power management to prevent system crashes
- Support for AXP2101 chip ID variants (0x47, 0x4A)
- AW9523 GPIO expander support for CoreS3

*Note: This component provides core functionality for the AXP2101. Additional features can be added as needed.*

## Installation

Copy the `components` directory to your ESPHome project, or include directly from this repository.

## Configuration

### Basic Component Setup

First, define the main AXP2101 component with your device model:

```yaml
i2c:
  - id: bus_a
    sda: GPIO21
    scl: GPIO22
    scan: true

axp2101:
  id: axp2101_component
  model: M5CORE2  # Use M5CORES3 for CoreS3
  address: 0x34
  i2c_id: bus_a
  update_interval: 100ms  # Use 100ms for responsive power button detection
```

**Model Selection:**
- `M5CORE2` - For M5Stack Core2 V1.1 (uses BLDO1 for backlight)
- `M5CORES3` - For M5Stack CoreS3 (uses DLDO1 for backlight)

### Battery Monitoring

Add sensor platform for battery metrics:

```yaml
sensor:
  - platform: axp2101
    axp2101_id: axp2101_component
    battery_voltage:
      name: "Battery Voltage"
    battery_level:
      name: "Battery Level"
    temperature:
      name: "PMU Temperature"
```

**Available Sensors:**
- `battery_voltage` - Battery voltage in volts (V)
- `battery_level` - Battery level percentage (0-100%)
- `temperature` - Internal PMU die temperature in °C

**Note:** VBUS (USB power) monitoring is not available on M5Stack Core2 V1.1 hardware as USB power does not route through the AXP2101's VBUS detection circuitry.

### Binary Sensors

Add binary sensors for charging status and power button events:

```yaml
binary_sensor:
  - platform: axp2101
    axp2101_id: axp2101_component
    battery_charging:
      name: "Battery Charging"
    vbus_connected:
      name: "USB Connected"
    low_battery_level1:
      name: "Low Battery Warning"
      on_press:
        - logger.log: "Low battery warning!"
    low_battery_level2:
      name: "Critical Battery Warning"
      on_press:
        - logger.log: "Critical battery!"
    pkey_short_press:
      name: "Power Button Short Press"
      on_press:
        - logger.log: "Power button short press!"
    pkey_long_press:
      name: "Power Button Long Press"
      on_press:
        - logger.log: "Power button long press!"
    # Optional: Track button state with positive/negative edges
    # pkey_positive:
    #   name: "Power Button Pressed"
    # pkey_negative:
    #   name: "Power Button Released"
```

**Available Binary Sensors:**
- `battery_charging` - On when battery is charging
- `vbus_connected` - On when USB power is connected
- `low_battery_level1` - Warning when battery drops below ~15%
- `low_battery_level2` - Critical warning when battery drops below ~5%
- `pkey_short_press` - Momentary trigger on short button press
- `pkey_long_press` - Momentary trigger on long button press (typically 1-2 seconds)
- `pkey_positive` - On when button is pressed down (optional)
- `pkey_negative` - On when button is released (optional)

**Low Battery Warnings:**
The low battery sensors use hardware interrupts from the AXP2101 to detect when the battery level drops below predefined thresholds (~15% and ~5%). These sensors automatically clear when the battery is charged above the threshold. Use these with automation to trigger warnings or power-saving modes.

**Power Button Events:**
The power button binary sensors are momentary - they trigger briefly when the event occurs and then reset. This makes them ideal for automation triggers. Use `on_press` actions to respond to button events.

For tracking the actual button state (pressed vs released), use the optional `pkey_positive` and `pkey_negative` sensors.

**Note:** For responsive button detection, set `update_interval` to 100ms or faster. The default 60s interval will cause button events to accumulate and trigger all at once.

### Brightness and LED Control

Add number platform for brightness and charging LED control:

```yaml
number:
  - platform: axp2101
    axp2101_id: axp2101_component
    backlight:
      name: "Backlight Brightness"
    charging_led_mode:
      name: "Charging LED Mode"
```

**Available Number Entities:**
- `backlight` - Display backlight brightness slider (0-100%) - **Works for both M5Core2 and CoreS3**
  - M5Core2: Controls BLDO1 voltage (2500-3300mV)
  - CoreS3: Controls DLDO1 voltage (500-3400mV)
- `charging_led_mode` - Charging LED behavior control (0-4)
  - 0: Off
  - 1: Blink 1Hz
  - 2: Blink 4Hz
  - 3: On (solid)
  - 4: Auto (controlled by charger)

The backlight creates a 0-100% slider in Home Assistant that controls the display backlight brightness in real-time. The component automatically uses the appropriate regulator (BLDO1 or DLDO1) based on the model specified in the main axp2101 component configuration.

### Complete Example

See `/sample-config` folder for complete device configurations:
- `m5core2.yaml` - M5Stack Core2 V1.1 configuration
- `m5coreS3.yaml` - M5Stack CoreS3 configuration with AW9523 GPIO expander

### AW9523 GPIO Expander (CoreS3)

The M5Stack CoreS3 uses an AW9523 GPIO expander for additional I/O:

```yaml
aw9523:
  id: aw9523_component
  address: 0x58
  i2c_id: bus_a
```

**CoreS3 Pin Usage:**
- `P1_1` - LCD reset pin
- `P1_7` - BOOST_EN (display boost enable)

The AW9523 component automatically configures these pins and monitors BOOST_EN to ensure the display remains powered.

### Display Configuration

The M5Stack devices use ILI9xxx displays:

**M5Stack Core2 V1.1:**
```yaml
display:
  - platform: ili9xxx
    model: M5STACK
    dimensions: 320x240
    cs_pin: GPIO5
    dc_pin: GPIO15
```

**M5Stack CoreS3:**
```yaml
display:
  - platform: ili9xxx
    model: ILI9342
    dimensions: 320x240
    cs_pin: GPIO3
    dc_pin: GPIO35
    reset_pin:
      aw9523: aw9523_component
      number: 9  # P1_1
      inverted: true
    invert_colors: true
    data_rate: 8MHz
```

Note: CoreS3 uses the AW9523 GPIO expander pin P1_1 for LCD reset.

## Technical Details

### AXP2101 Power Management
- **I2C Address:** 0x34
- **Chip IDs:** 0x47, 0x4A (both supported)
- **M5Core2 Backlight:** BLDO1 regulator (2500-3300mV)
- **CoreS3 Backlight:** DLDO1 regulator (500-3400mV, 100mV steps)
- **Battery ADC:** 14-bit resolution, 1.1mV LSB
- **VBUS ADC:** 14-bit resolution, 1.1mV LSB
- **Temperature Range:** Internal die temperature monitoring
- **Framework:** ESP-IDF (no Arduino dependencies)

### AW9523 GPIO Expander (CoreS3)
- **I2C Address:** 0x58
- **Pins:** 16 GPIO pins (P0_0 to P0_7, P1_0 to P1_7)
- **Modes:** GPIO input, GPIO output, LED mode (PWM)
- **CoreS3 Usage:** P1_1 (LCD reset), P1_7 (BOOST_EN)

## Features

### Power Monitoring
- Battery voltage and level with 14-bit ADC precision
- Charging status detection
- USB power connection detection
- Internal temperature monitoring for thermal management

**Note:** USB/VBUS monitoring is not supported on M5Stack Core2 V1.1 as the hardware does not route USB power through the AXP2101's VBUS detection pins.

### Power Button Events
- Short press detection (momentary trigger)
- Long press detection (momentary trigger, typically 1-2 seconds)
- Button state tracking (pressed/released)
- Interrupt-based detection for reliable event capture

### Low Battery Warnings
- Level 1 warning (~15% battery) - Early warning
- Level 2 warning (~5% battery) - Critical warning
- Hardware interrupt-based detection
- Auto-clears when battery is recharged

### Charging LED Control
- 5 modes: Off, Blink 1Hz, Blink 4Hz, On (solid), Auto
- Manual control or automatic control by charger
- Real-time mode switching via Home Assistant

### Backlight Control
- **M5Core2:** Adjustable brightness (0-100%) via number platform using BLDO1 (2500-3300mV)
- **CoreS3:** Adjustable brightness (0-100%) via number platform using DLDO1 (500-3400mV)
- Real-time brightness adjustment
- Model-specific voltage control (automatically selected based on model parameter)

### Power Management
- Conservative initialization to prevent system crashes
- Support for multiple chip ID variants
- Direct I2C register access for reliability
