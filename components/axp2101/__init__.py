import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_MODEL

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor", "number"]

CONF_AXP2101_ID = "axp2101_id"

axp2101_ns = cg.esphome_ns.namespace("axp2101")
AXP2101Component = axp2101_ns.class_(
    "AXP2101Component", cg.PollingComponent, i2c.I2CDevice
)

AXP2101Model = axp2101_ns.enum("AXP2101Model")

MODELS = {
    "M5CORE2": AXP2101Model.AXP2101_M5CORE2,
    "M5CORES3": AXP2101Model.AXP2101_M5CORES3,
}

AXP2101_MODEL = cv.enum(MODELS, upper=True, space="_")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AXP2101Component),
            cv.Required(CONF_MODEL): AXP2101_MODEL,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x34))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_model(config[CONF_MODEL]))
