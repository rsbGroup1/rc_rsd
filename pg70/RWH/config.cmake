# -*- cmake -*-
#
# ROBWORKHARDWARE SPECIFIC
#

# specify any of these options to force the inclusion of the specific component 
SET(COMPONENT_camera_ENABLE OFF)
SET(COMPONENT_CAN_ENABLE OFF)
SET(COMPONENT_CRSA465_ENABLE OFF)
SET(COMPONENT_DOCKWELDER_ENABLE OFF)
SET(COMPONENT_fanucdevice_ENABLE OFF)
SET(COMPONENT_katana_ENABLE OFF)
SET(COMPONENT_motomanIA20_ENABLE OFF)
SET(COMPONENT_netft_ENABLE OFF)
SET(COMPONENT_pa10_ENABLE OFF)
SET(COMPONENT_pcube_ENABLE ON)
SET(COMPONENT_schunkpg70_ENABLE ON)
SET(COMPONENT_sdh_ENABLE OFF)
SET(COMPONENT_serialport_ENABLE ON)
SET(COMPONENT_sick_ENABLE OFF)
SET(COMPONENT_swissranger_ENABLE OFF)
SET(COMPONENT_tactile_ENABLE OFF)
SET(COMPONENT_trakstar_ENABLE OFF)
SET(COMPONENT_universalrobots_ENABLE OFF)
SET(COMPONENT_robolabFT_ENABLE OFF)

# we need the location of RobWork
SET(RW_ROOT ${ROOT}/../RobWork/)

# Instead of setting the individual variables of robwork you could 
# use the same options as in the RobWork project
INCLUDE(${RW_ROOT}/config.cmake OPTIONAL)

# Enable esdcan driver
#SET(ESDCAN_INCLUDE_DIR "C:/Program Files/ESD/CAN/SDK/include")
#SET(ESDCAN_LIB_DIR "C:/Program Files/ESD/CAN/SDK/lib/vc/i386")

# Enable katana driver
#SET(KATANA_INCLUDE_DIR "${ROOT}/ext/katana/include/")
#SET(KATANA_LIB_DIR "${ROOT}/ext/katana/lib/")

# Enable SDH driver
#SET(SDH_INCLUDE_DIR "${ROOT}/ext/sdh2/")
#SET(SDH_LIB_DIR "${ROOT}/ext/sdh2/libs/")

