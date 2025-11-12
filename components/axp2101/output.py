"""AXP2101 Output Platform - DLDO1 Backlight Control"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_ID
from . import AXP2101Component, CONF_AXP2101_ID

DEPENDENCIES = ["axp2101"]

axp2101_ns = cg.esphome_ns.namespace("axp2101")
AXP2101Output = axp2101_ns.class_("AXP2101Output", output.FloatOutput, cg.Component)

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(AXP2101Output),
        cv.GenerateID(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await output.register_output(var, config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_AXP2101_ID])
    cg.add(var.set_parent(parent))
