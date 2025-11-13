import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_BATTERY_CHARGING,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_BATTERY,
    ICON_FLASH,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
)
from . import axp2101_ns, AXP2101Component, CONF_AXP2101_ID

DEPENDENCIES = ["axp2101"]

CONF_BATTERY_CHARGING = "battery_charging"
CONF_VBUS_VOLTAGE = "vbus_voltage"
CONF_VBUS_CONNECTED = "vbus_connected"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                device_class=DEVICE_CLASS_VOLTAGE,
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon=ICON_BATTERY,
            ),
            cv.Optional(CONF_BATTERY_CHARGING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_BATTERY_CHARGING,
            ),
            cv.Optional(CONF_VBUS_VOLTAGE): sensor.sensor_schema(
                device_class=DEVICE_CLASS_VOLTAGE,
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VBUS_CONNECTED): binary_sensor.binary_sensor_schema(
                icon=ICON_FLASH,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                device_class=DEVICE_CLASS_TEMPERATURE,
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AXP2101_ID])

    if CONF_BATTERY_VOLTAGE in config:
        conf = config[CONF_BATTERY_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_batteryvoltage_sensor(sens))

    if CONF_BATTERY_LEVEL in config:
        conf = config[CONF_BATTERY_LEVEL]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_batterylevel_sensor(sens))

    if CONF_BATTERY_CHARGING in config:
        conf = config[CONF_BATTERY_CHARGING]
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(parent.set_batterycharging_bsensor(sens))

    if CONF_VBUS_VOLTAGE in config:
        conf = config[CONF_VBUS_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_vbusvoltage_sensor(sens))

    if CONF_VBUS_CONNECTED in config:
        conf = config[CONF_VBUS_CONNECTED]
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(parent.set_vbusconnected_bsensor(sens))

    if CONF_TEMPERATURE in config:
        conf = config[CONF_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_temperature_sensor(sens))
