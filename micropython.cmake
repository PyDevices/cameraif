# cameraif: MIPI-CSI camera capture as a MicroPython native module.
#
# For Makefile-based ports there is deliberately no micropython.mk: every
# line of this module is ESP32-P4 CSI hardware, so there is nothing a unix
# or rp2 build could compile. The Python-facing functions still exist on
# those ports -- mod_cameraif.c guards its bodies -- so an application can
# ask `cameraif.available()` rather than catching ImportError.
set(CAMERAIF_MOD_DIR ${CMAKE_CURRENT_LIST_DIR})
set(CAMERAIF_SRC_DIR ${CAMERAIF_MOD_DIR}/src)

add_library(usermod_cameraif INTERFACE)

target_sources(usermod_cameraif INTERFACE
    ${CAMERAIF_SRC_DIR}/mod_cameraif.c
)

target_include_directories(usermod_cameraif INTERFACE
    ${CAMERAIF_SRC_DIR}
)

# The IDF components this module drives. Declared rather than assumed: a
# usermod is an INTERFACE library added to `main`, so it only sees what main
# already required -- esp_driver_cam and esp_driver_jpeg are not in that set,
# and without these the headers are simply not on the include path.
target_link_libraries(usermod_cameraif INTERFACE
    idf::esp_driver_cam
    idf::esp_driver_isp
    idf::esp_driver_jpeg
)

# Note there is deliberately no `-u ov5647_detect` here.
#
# Sensor drivers register their probe in a linker section that nothing in C
# ever names, so --gc-sections would drop them -- but esp_cam_sensor's own
# CMakeLists already emits the `-u` when CONFIG_CAMERA_OV5647_AUTO_DETECT_
# MIPI_INTERFACE_SENSOR is set. Adding it here as well looked harmless and
# was not: with CONFIG_CAMERA_OV5647 unset the driver is not compiled at all,
# so the forced reference resolved to nothing, the link still succeeded, and
# the detect array was empty with `ov5647_detect` left undefined in the ELF.
# The camera then reports "no sensor answered on SCCB" -- a wiring-shaped
# error for a build-configuration cause. The sdkconfig options are what
# matter; they live in the cameraif patch.

target_link_libraries(usermod INTERFACE usermod_cameraif)
