from zigpy.quirks import CustomCluster
from zigpy.quirks.v2 import QuirkBuilder, ReportingConfig
from zigpy.quirks.v2.homeassistant import EntityType
from zigpy.zcl.clusters.general import AnalogInput, MultistateValue, BinaryInput
import zigpy.types as t
from zigpy.quirks.v2.homeassistant.sensor import SensorDeviceClass, SensorStateClass

from zha.application.platforms.number.const import NumberMode
from zigpy.quirks.v2.homeassistant.number import NumberDeviceClass

import functools
from zha.application import Platform
from zha.application.registries import PLATFORM_ENTITIES
from zha.application.platforms.sensor import Sensor
from zha.zigbee.cluster_handlers.const import CLUSTER_HANDLER_ANALOG_INPUT
from zha.zigbee.cluster_handlers import ClusterHandler

from zigpy.quirks.v2 import CustomDeviceV2
from zha.zigbee.cluster_handlers.const import REPORT_CONFIG_IMMEDIATE
from zha.zigbee.cluster_handlers import (
    AttrReportConfig,
    registries,
)
from zha.zigbee.cluster_handlers.general import AnalogInputClusterHandler, BinaryInputClusterHandler


DS_PRESENCE_QUIRK_ID = "ds_presence1_quirk"

#
# Increase reporting rate for the input clusters: we need to process motion detection immediately
#
@registries.CLUSTER_HANDLER_REGISTRY.register(
    BinaryInput.cluster_id, DS_PRESENCE_QUIRK_ID
)
class dsBinaryInputClusterHandler(BinaryInputClusterHandler):
    REPORT_CONFIG = (
        AttrReportConfig(attr="present_value", config=REPORT_CONFIG_IMMEDIATE),
    )

@registries.CLUSTER_HANDLER_REGISTRY.register(
    AnalogInput.cluster_id, DS_PRESENCE_QUIRK_ID
)
class dsAnalogInputClusterHandler(AnalogInputClusterHandler):
    REPORT_CONFIG = (
        AttrReportConfig(attr="present_value", config=REPORT_CONFIG_IMMEDIATE),
    )



CALIB_TIME=100


#
# Apply the quirk
#
class dsPresenceQuirkDevice(CustomDeviceV2):
    quirk_id = DS_PRESENCE_QUIRK_ID

(
    QuirkBuilder("DS", "Presence1")
    .device_class(dsPresenceQuirkDevice)
    .write_attr_button(
        attribute_name = "present_value",
        attribute_value = CALIB_TIME,
        cluster_id = AnalogInput.cluster_id,
        endpoint_id = 2,
        fallback_name = "Calibrate",
        translation_key = "calibrate",
    )
    .add_to_registry()
)


