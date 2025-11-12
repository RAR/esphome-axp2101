import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
    ICON_BRIGHTNESS_6,
    UNIT_PERCENT,
)
from . import AXP2101Component, axp2101_ns

DEPENDENCIES = ["axp2101"]

AXP2101Number = axp2101_ns.class_("AXP2101Number", number.Number, cg.Component)

CONF_AXP2101_ID = "axp2101_id"
CONF_BACKLIGHT = "backlight"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
        cv.Optional(CONF_BACKLIGHT): number.number_schema(
            AXP2101Number,
            icon=ICON_BRIGHTNESS_6,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_PERCENT,
        ).extend(
            {
                cv.Optional("initial_value", default=100.0): cv.float_range(min=0, max=100),
                cv.Optional("restore_value", default=True): cv.boolean,
            }
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AXP2101_ID])
    
    if CONF_BACKLIGHT in config:
        conf = config[CONF_BACKLIGHT]
        num = await number.new_number(conf, min_value=0, max_value=100, step=5)
        await cg.register_component(num, conf)
        cg.add(num.set_parent(parent))
        cg.add(parent.set_brightness_number(num))
