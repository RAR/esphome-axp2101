# AXP2101 Component - ESP-IDF Rewrite

## Overview
This component has been completely rewritten to work with ESP-IDF instead of Arduino framework. The original implementation relied on the XPowersLib Arduino library, which is not compatible with ESP-IDF.

## Key Changes

### 1. Removed Arduino Dependencies
- **Removed**: XPowersLib dependency
- **Replaced with**: Direct I2C register access using ESPHome's I2C device class

### 2. Register-Level Implementation
The new implementation directly communicates with the AXP2101 via I2C registers:
- All power management functionality now uses direct register reads/writes
- Added comprehensive register definitions in `axp2101.h`
- Implemented helper methods for register access (`writeRegister`, `readRegister`, `setBits`, `clearBits`)

### 3. Key Features Implemented

#### Power Output Control
- DC-DC converters (DC1-DC5) configuration
- LDO regulators (ALDO1-4, BLDO1-2, CPUSLDO, DLDO1-2)
- Voltage setting with proper range handling
- Enable/disable control for each output

#### Battery Monitoring
- Battery voltage reading (14-bit ADC, 1.1mV resolution)
- Battery percentage from internal fuel gauge
- Charging status detection
- Battery connection detection

#### Charging Control
- Charge current configuration
- Charge voltage setting
- Termination current setting
- VBUS voltage and current limits
- TS pin handling (disabled for boards without temperature sensor)

#### Interrupt Handling
- ESP-IDF compatible GPIO interrupt setup
- Interrupt status reading and clearing
- Configurable interrupt sources (battery, VBUS, power key, charging)

#### Backlight Control
- Voltage-based brightness control via BLDO1
- Smooth brightness transitions
- Configurable brightness levels (2.5V-3.3V range)

### 4. Files Modified

#### `sensor.py` (Python Configuration)
- Removed `cg.add_library("lewisxhe/XPowersLib", "0.3.0")`
- Component now works with both Arduino and ESP-IDF frameworks

#### `axp2101.h` (Header File)
- Removed `#include "XPowersLib.h"`
- Added complete register map for AXP2101
- Simplified class interface
- Added helper methods for register operations
- Removed deprecated methods

#### `axp2101.cpp` (Implementation)
- Complete rewrite using ESP-IDF compatible APIs
- Direct register-level I2C communication
- Proper GPIO interrupt handling for ESP-IDF
- Organized into initialization functions:
  - `initPowerOutputs()` - Configure DC-DC and LDO outputs
  - `initCharger()` - Setup battery charging parameters
  - `initInterrupts()` - Configure interrupt handling

### 5. Register Definitions Added

Key register addresses defined:
- `AXP2101_CHIP_ID` (0x03) - Chip identification
- `AXP2101_COMM_STAT0/1` (0x01-0x02) - Status registers
- `AXP2101_DC_VOL0-4` (0x82-0x86) - DC voltage control
- `AXP2101_ALDO_VOL0-3` (0x92-0x95) - ALDO voltage control
- `AXP2101_BLDO_VOL0-1` (0x96-0x97) - BLDO voltage control
- `AXP2101_VBAT_H/L` (0x34-0x35) - Battery voltage
- `AXP2101_BAT_PERCENT` (0xA4) - Battery percentage
- `AXP2101_IRQ_EN0-2` (0x40-0x42) - Interrupt enable
- `AXP2101_IRQ_STATUS0-2` (0x48-0x4A) - Interrupt status

### 6. Compatibility

#### Tested Platforms
- M5Stack Core2 (with appropriate sample configuration)
- M5Stack Core S3 (with appropriate sample configuration)

#### Framework Support
- **ESP-IDF**: ✅ Fully supported (primary target)
- **Arduino**: ✅ Should still work (uses ESPHome I2C abstraction)

### 7. Configuration Changes

No configuration changes required in YAML files. The component maintains the same API:

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
```

To use ESP-IDF framework, update your ESP32 configuration:

```yaml
esp32:
  board: m5stack-core2
  framework:
    type: esp-idf  # Changed from 'arduino'
```

## Technical Details

### Voltage Calculations

#### DC-DC Converters
- **DC1**: 1500-3400mV, 100mV/step (20 steps)
- **DC2-4**: 500-1200mV (10mV/step, 71 steps), 1220-1540mV (20mV/step, 17 steps)
- **DC5**: 1200mV or 1400-3700mV (100mV/step, 24 steps)

#### LDO Regulators
- **ALDO1-4**: 500-3500mV, 100mV/step (31 steps)
- **BLDO1-2**: 500-3500mV, 100mV/step (31 steps)
- **CPUSLDO**: 500-1400mV, 50mV/step (19 steps)

#### Battery ADC
- **Resolution**: 14-bit
- **LSB**: 1.1mV
- **Range**: ~0-18V (theoretical)

### Interrupt System

The component uses GPIO interrupts to monitor AXP2101 events:
- IRQ pin configured as input with pull-up
- Falling edge interrupt detection
- ISR handler sets flag for processing in update loop
- Interrupt sources: battery insert/remove, VBUS changes, power key, charging events

### Power Management

Default configuration for M5Stack Core2:
- **DC1**: 3.3V (Internal, may be pre-enabled)
- **DC2**: 1.0V (Internal logic)
- **DC3**: 3.3V (Display power)
- **DC4**: 1.0V (Internal logic)
- **DC5**: 3.3V (External peripherals)
- **ALDO1-2**: 3.3V (Various)
- **ALDO3**: Disabled (Speaker control)
- **ALDO4**: 3.3V (Display)
- **BLDO1**: 2.5-3.3V (Backlight, voltage-controlled)
- **BLDO2**: 3.3V
- **CPUSLDO**: 1.0V (CPU LDO)

## Migration Notes

If you were using the Arduino version:
1. No code changes required in your YAML configuration
2. Change framework to `esp-idf` in your ESP32 config
3. Recompile and flash

The new implementation provides the same functionality with better ESP-IDF integration.

## Known Issues / Limitations

1. Battery fuel gauge may be inaccurate on first use - requires one full charge/discharge cycle for calibration
2. Some register addresses may need verification against specific AXP2101 variants
3. DLDO1/DLDO2 (vibration motor, etc.) are not configured by default - add if needed

## Future Enhancements

Potential additions:
- Coulomb counter support
- Temperature monitoring
- Power button press detection
- More granular charging control
- VBUS/VIN current monitoring
- Power path management

## Testing

To test the new implementation:
1. Build with ESP-IDF framework
2. Monitor logs for "Chip ID: 0x47" on startup
3. Verify battery voltage readings
4. Check charging status changes when USB connected/disconnected
5. Test backlight brightness control

## Backup

The original Arduino-based implementation has been backed up as `axp2101.cpp.old` for reference.
