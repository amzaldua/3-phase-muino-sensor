import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

CODEOWNERS = ["@martijnvwezel", "@hugokernel"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

# Create the namespace for this component
muino_3phase_i2c_ns = cg.esphome_ns.namespace("muino_3phase_i2c")
Muino3PhaseI2CSensor = muino_3phase_i2c_ns.class_("Muino3PhaseI2CSensor", cg.Component, i2c.I2CDevice)

# This is important - define the base component configuration
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Muino3PhaseI2CSensor),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
