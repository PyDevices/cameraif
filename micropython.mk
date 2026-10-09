# cameraif on Makefile-built firmware, which here means CircuitPython.
#
# MicroPython's Makefile ports (unix, rp2, ...) get nothing from this file:
# every line of cameraif is ESP32-P4 CSI hardware, and MicroPython's esp32 port
# builds it through micropython.cmake. CircuitPython builds every port with
# make, its espressif port included, so this is its route in:
#
#   make -C ports/espressif BOARD=<an ESP32-P4 board> USER_C_MODULES=/path/to/cameraif \
#       USER_ESP_IDF_COMPONENT_DIRS="/path/to/esp_cam_sensor /path/to/esp_sccb_intf /path/to/cmake_utilities"
#
# That needs CircuitPython's espressif port to build ESP-IDF components a
# module asks for (USER_ESP_IDF_COMPONENTS), which it doesn't on its own;
# micropython-pydevices carries the patch, and its build_mp.py fetches the
# three components at their pinned versions and passes them. The board's
# sdkconfig has to enable a sensor driver and its auto-detection, for example
# CONFIG_CAMERA_OV5647=y and CONFIG_CAMERA_OV5647_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y;
# without them every camera reports as absent.
#
# On any other CircuitPython target the module still builds, and
# cameraif.available() says False.

ifneq ($(wildcard $(TOP)/py/circuitpy_mpconfig.h),)

CAMERAIF_MOD_DIR := $(USERMOD_DIR)

SRC_USERMOD_C += $(CAMERAIF_MOD_DIR)/src/mod_cameraif.c
CFLAGS_USERMOD += -I$(CAMERAIF_MOD_DIR)/src

# Which cameraif this firmware was built from: cameraif.__revision__.
CAMERAIF_REVISION := $(shell git -C $(CAMERAIF_MOD_DIR) describe --always --dirty --abbrev=7 2>/dev/null || echo unknown)
CFLAGS_USERMOD += -DCAMERAIF_REVISION='"$(CAMERAIF_REVISION)"'

ifeq ($(IDF_TARGET),esp32p4)
CAMERAIF_CAM_SENSOR := $(filter %/esp_cam_sensor,$(USER_ESP_IDF_COMPONENT_DIRS:/=))
CAMERAIF_SCCB := $(filter %/esp_sccb_intf,$(USER_ESP_IDF_COMPONENT_DIRS:/=))
ifeq ($(CAMERAIF_CAM_SENSOR),)
$(error cameraif: USER_ESP_IDF_COMPONENT_DIRS names no esp_cam_sensor. ESP-IDF has no camera sensor drivers; build with micropython-pydevices' build_mp.py, or pass the esp_cam_sensor, esp_sccb_intf and cmake_utilities component directories)
endif
ifeq ($(CAMERAIF_SCCB),)
$(error cameraif: USER_ESP_IDF_COMPONENT_DIRS names no esp_sccb_intf, which esp_cam_sensor needs)
endif

# The CSI controller, the ISP behind it and their HAL, the sensor drivers and
# their SCCB layer: each builds a library, and the port links them.
USER_ESP_IDF_COMPONENTS += esp_driver_cam esp_driver_isp esp_hal_cam esp_cam_sensor esp_sccb_intf

# CircuitPython compiles modules outside ESP-IDF's CMake, so the components'
# include directories are named here.
CAMERAIF_IDF := $(TOP)/ports/espressif/esp-idf/components
CFLAGS_USERMOD += \
	-isystem $(CAMERAIF_IDF)/esp_driver_cam/include \
	-isystem $(CAMERAIF_IDF)/esp_driver_cam/interface \
	-isystem $(CAMERAIF_IDF)/esp_driver_cam/csi/include \
	-isystem $(CAMERAIF_IDF)/esp_driver_isp/include \
	-isystem $(CAMERAIF_IDF)/esp_hal_cam/include \
	-isystem $(CAMERAIF_IDF)/esp_hal_cam/esp32p4/include \
	-isystem $(CAMERAIF_CAM_SENSOR)/include \
	-isystem $(CAMERAIF_SCCB)/include \
	-isystem $(CAMERAIF_SCCB)/interface \
	-isystem $(CAMERAIF_SCCB)/sccb_i2c/include
endif

endif
