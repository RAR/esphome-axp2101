import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_POWER

from . import AXP2101Component, axp2101_ns

DEPENDENCIES = ["axp2101"]

CONF_AXP2101_ID = "axp2101_id"
CONF_BATTERY_CHARGING = "battery_charging"
CONF_VBUS_CONNECTED = "vbus_connected"
CONF_PKEY_SHORT_PRESS = "pkey_short_press"
CONF_PKEY_LONG_PRESS = "pkey_long_press"
CONF_PKEY_POSITIVE = "pkey_positive"
CONF_PKEY_NEGATIVE = "pkey_negative"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
        cv.Optional(CONF_BATTERY_CHARGING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER
        ),
        cv.Optional(CONF_VBUS_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER
        ),
        cv.Optional(CONF_PKEY_SHORT_PRESS): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_PKEY_LONG_PRESS): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_PKEY_POSITIVE): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_PKEY_NEGATIVE): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    axp = await cg.get_variable(config[CONF_AXP2101_ID])

    if battery_charging_config := config.get(CONF_BATTERY_CHARGING):
        sens = await binary_sensor.new_binary_sensor(battery_charging_config)
        cg.add(axp.set_batterycharging_bsensor(sens))

    if vbus_connected_config := config.get(CONF_VBUS_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(vbus_connected_config)
        cg.add(axp.set_vbusconnected_bsensor(sens))

    if pkey_short_press_config := config.get(CONF_PKEY_SHORT_PRESS):
        sens = await binary_sensor.new_binary_sensor(pkey_short_press_config)
        cg.add(axp.set_pkey_short_bsensor(sens))

    if pkey_long_press_config := config.get(CONF_PKEY_LONG_PRESS):
        sens = await binary_sensor.new_binary_sensor(pkey_long_press_config)
        cg.add(axp.set_pkey_long_bsensor(sens))

    if pkey_positive_config := config.get(CONF_PKEY_POSITIVE):
        sens = await binary_sensor.new_binary_sensor(pkey_positive_config)
        cg.add(axp.set_pkey_positive_bsensor(sens))

    if pkey_negative_config := config.get(CONF_PKEY_NEGATIVE):
        sens = await binary_sensor.new_binary_sensor(pkey_negative_config)
        cg.add(axp.set_pkey_negative_bsensor(sens))
