# ESPHome AXP2101 Component

This custom component provides **ESP-IDF native** support for the AXP2101 power management IC used in the [M5Stack Core2 V1.1](https://docs.m5stack.com/en/core/Core2%20v1.1). This is a complete rewrite using direct I2C register access instead of Arduino libraries, making it compatible with ESP-IDF framework projects.

**Key Features:**
- ESP-IDF native implementation (no Arduino framework required)
- Direct I2C register access (no XPowersLib dependency)
- Battery monitoring (voltage, level, charging status)
- Backlight brightness control with Home Assistant integration
- Conservative power management to prevent system crashes
- Support for AXP2101 chip ID variants (0x47, 0x4A)

*Note: This component provides core functionality for the AXP2101. Additional features can be added as needed.*

## Installation

Copy the `components` directory to your ESPHome project, or include directly from this repository.

## Configuration

### Basic Component Setup

First, define the main AXP2101 component:

```yaml
i2c:
  - id: bus_a
    sda: GPIO21
    scl: GPIO22
    scan: true

axp2101:
  id: axp2101_component
  update_interval: 60s
```

### Battery Monitoring

Add sensor platform for battery metrics:

```yaml
sensor:
  - platform: axp2101
    model: M5CORE2
    address: 0x34
    i2c_id: bus_a
    update_interval: 30s
    brightness: 75%
    battery_voltage:
      name: "Battery Voltage"
    battery_level:
      name: "Battery Level"
    battery_charging:
      name: "Battery Charging"
    temperature:
      name: "PMU Temperature"
```

**Available Sensors:**
- `battery_voltage` - Battery voltage in volts (V)
- `battery_level` - Battery level percentage (0-100%)
- `battery_charging` - Binary sensor indicating charging status
- `temperature` - Internal PMU die temperature in °C

**Note:** VBUS (USB power) monitoring is not available on M5Stack Core2 V1.1 hardware as USB power does not route through the AXP2101's VBUS detection circuitry.

### Brightness Control (Home Assistant)

Add number platform for interactive brightness control:

```yaml
number:
  - platform: axp2101
    axp2101_id: axp2101_component
    backlight:
      name: "Backlight Brightness"
```

This creates a 0-100% slider in Home Assistant that controls the display backlight brightness in real-time.

### Complete Example

See `/sample-config` folder for complete device configurations.

### Display Configuration

The M5Stack Core2 V1.1 uses an ILI9341 display:

```yaml
font:
  - file: "gfonts://Roboto"
    id: roboto
    size: 24

display:
  - platform: ili9xxx
    model: M5STACK
    dimensions: 320x240
    cs_pin: GPIO5
    dc_pin: GPIO15
    lambda: |-
      it.print(0, 0, id(roboto), "Hello World");
```

## Technical Details

- **I2C Address:** 0x34
- **Chip IDs:** 0x47, 0x4A (both supported)
- **Backlight Control:** BLDO1 regulator (2500-3300mV)
- **Battery ADC:** 14-bit resolution, 1.1mV LSB
- **VBUS ADC:** 14-bit resolution, 1.1mV LSB
- **Temperature Range:** Internal die temperature monitoring
- **Framework:** ESP-IDF (no Arduino dependencies)

## Features

### Power Monitoring
- Battery voltage and level with 14-bit ADC precision
- Charging status detection
- Internal temperature monitoring for thermal management

**Note:** USB/VBUS monitoring is not supported on M5Stack Core2 V1.1 as the hardware does not route USB power through the AXP2101's VBUS detection pins.

### Backlight Control
- Adjustable brightness (0-100%) via Home Assistant
- Voltage-based control (2500-3300mV)
- Real-time brightness adjustment

### Power Management
- Conservative initialization to prevent system crashes
- Support for multiple chip ID variants
- Direct I2C register access for reliability
