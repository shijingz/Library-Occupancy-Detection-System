PROJECT_NAME     := ble_app_multilink_central_pca10040_s132
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../nRF5_SDK_15.0.0_a53641a
MESH_SDK_ROOT := ../../../../nrf5_SDK_for_Mesh_v2.2.0_src
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := nrf52832_xxAA_s132_6.0.0.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_btn_ble.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/ble_db_discovery/ble_db_discovery.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c/ble_lbs_c.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/config_server.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/composition_data.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/packed_index_list.c \
  $(MESH_SDK_ROOT)/models/foundation/health/src/health_server.c \
  $(MESH_SDK_ROOT)/models/model_spec/common/src/model_common.c \
  $(MESH_SDK_ROOT)/external/micro-ecc/uECC.c \
  $(MESH_SDK_ROOT)/examples/common/src/assertion_handler_weak.c \
  $(MESH_SDK_ROOT)/examples/common/src/mesh_provisionee.c \
  $(MESH_SDK_ROOT)/mesh/core/src/aes_cmac.c \
  $(MESH_SDK_ROOT)/mesh/core/src/aes.c \
  $(MESH_SDK_ROOT)/mesh/core/src/beacon.c \
  $(MESH_SDK_ROOT)/mesh/core/src/bearer_event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/ccm_soft.c \
  $(MESH_SDK_ROOT)/mesh/core/src/core_tx_adv.c \
  $(MESH_SDK_ROOT)/mesh/core/src/core_tx.c \
  $(MESH_SDK_ROOT)/mesh/core/src/enc.c \
  $(MESH_SDK_ROOT)/mesh/core/src/event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/fifo.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager_defrag.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager_internal.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager.c \
  $(MESH_SDK_ROOT)/mesh/core/src/fsm.c \
  $(MESH_SDK_ROOT)/mesh/core/src/hal.c \
  $(MESH_SDK_ROOT)/mesh/core/src/heartbeat.c \
  $(MESH_SDK_ROOT)/mesh/core/src/internal_event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/list.c \
  $(MESH_SDK_ROOT)/mesh/core/src/log.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config_backend.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config_flashman_glue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_flash.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_opt.c \
  $(MESH_SDK_ROOT)/mesh/core/src/msg_cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/msqueue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_beacon.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_packet.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_state.c \
  $(MESH_SDK_ROOT)/mesh/core/src/network.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_flash.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_configure.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_keygen.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_opt.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_utils.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh.c \
  $(MESH_SDK_ROOT)/mesh/core/src/packet_buffer.c \
  $(MESH_SDK_ROOT)/mesh/core/src/packet_mgr.c \
  $(MESH_SDK_ROOT)/mesh/core/src/queue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/rand.c \
  $(MESH_SDK_ROOT)/mesh/core/src/replay_cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timer_scheduler.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timer.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timeslot.c \
  $(MESH_SDK_ROOT)/mesh/core/src/toolchain.c \
  $(MESH_SDK_ROOT)/mesh/core/src/transport.c \
  $(MESH_SDK_ROOT)/mesh/core/src/uri.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/ad_listener.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/ad_type_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/adv_packet_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/advertiser.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/bearer_handler.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/broadcast.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/filter_engine.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/gap_address_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/radio_config.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/rssi_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/scanner.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/mesh_pa_lna.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/instaburst.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/instaburst_rx.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/instaburst_tx.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/adv_ext_tx.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/adv_ext_packet.c   \
  $(MESH_SDK_ROOT)/mesh/prov/src/nrf_mesh_prov.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_beacon.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_bearer_adv.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_provisionee.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_provisioner.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_utils.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/provisioning.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_loopback.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_publish.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_reliable.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access.c \
  $(MESH_SDK_ROOT)/mesh/access/src/device_state_manager.c \
  $(MESH_SDK_ROOT)/mesh/dfu/src/nrf_mesh_dfu.c \
  $(MESH_SDK_ROOT)/mesh/stack/src/mesh_stack.c \
  $(MESH_SDK_ROOT)/mesh/gatt/src/mesh_gatt.c \
  $(PROJ_DIR)/app_api_for_model/src/app_error_weak.c \
  $(PROJ_DIR)/app_api_for_model/src/app_onoff.c \
  $(PROJ_DIR)/app_api_for_model/src/assertion_handler_weak.c \
  $(PROJ_DIR)/app_api_for_model/src/mesh_adv.c \
  $(PROJ_DIR)/app_api_for_model/src/mesh_app_utils.c \
  $(PROJ_DIR)/app_api_for_model/src/mesh_provisionee.c \
  $(PROJ_DIR)/app_api_for_model/src/rtt_input.c \
  $(PROJ_DIR)/customized_model/src/generic_onoff_client.c \
  $(PROJ_DIR)/customized_model/src/generic_onoff_server.c \
  $(PROJ_DIR)/src/mesh2pi_uart.c \
  $(PROJ_DIR)/mesh_main.c \
  $(PROJ_DIR)/main.c \


# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/experimental_log/src \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/common \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/libraries/experimental_memobj \
  $(SDK_ROOT)/components/nfc/t4t_parser/cc_file \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/ble/ble_db_discovery \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/experimental_mpu \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/libraries/usbd/config \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/libraries/experimental_log \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/nfc/t4t_parser/apdu \
  $(SDK_ROOT)/components/libraries/util \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/nfc/t4t_parser/tlv \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/components/libraries/experimental_stack_guard \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/nfc/ndef/parser/record \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_ROOT)/components/nfc/t2t_parser \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/nfc/t4t_lib/hal_t4t \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/drivers_nrf/usbd \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/nfc/ndef/parser/message \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/twi_sensor \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/experimental_ringbuf \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_ROOT)/components/nfc/t2t_lib/hal_t2t \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(MESH_SDK_ROOT)/mesh/core/api \
  $(MESH_SDK_ROOT)/mesh/core/include \
  $(MESH_SDK_ROOT)/mesh/bearer/api \
  $(MESH_SDK_ROOT)/mesh/gatt/api \
  $(MESH_SDK_ROOT)/mesh/gatt/include \
  $(MESH_SDK_ROOT)/mesh/bearer/include \
  $(MESH_SDK_ROOT)/mesh/prov/api \
  $(MESH_SDK_ROOT)/mesh/prov/include \
  $(MESH_SDK_ROOT)/mesh/access/api \
  $(MESH_SDK_ROOT)/mesh/access/include \
  $(MESH_SDK_ROOT)/mesh/dfu/api \
  $(MESH_SDK_ROOT)/mesh/dfu/include \
  $(MESH_SDK_ROOT)/mesh/stack/api \
  $(MESH_SDK_ROOT)/models/foundation/config/include \
  $(MESH_SDK_ROOT)/models/foundation/health/include \
  $(MESH_SDK_ROOT)/models/model_spec/common/include/ \
  $(MESH_SDK_ROOT)/external/micro-ecc \
  $(PROJ_DIR)/app_api_for_model/include \
  $(PROJ_DIR)/customized_model/include \
  $(PROJ_DIR)/include \


# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52
CFLAGS += -DNRF52832
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0

#CFLAGS += -DNO_VTOR_CONFIG
#CFLAGS += -DINITIALIZE_USER_SECTIONS

CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
# CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

CFLAGS += -DNRF52_SERIES
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DNRF_MESH_LOG_ENABLE=NRF_LOG_USES_RTT
CFLAGS += -DCONFIG_APP_IN_CORE
CFLAGS += -DUSE_APP_CONFIG
#CFLAGS += -DMESH_FEATURE_GATT=1
#CFLAGS += -DGATT_PROXY=1

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS132
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0

#ASMFLAGS += -DNO_VTOR_CONFIG
#ASMFLAGS += -DINITIALIZE_USER_SECTIONS

ASMFLAGS += -DNRF52_SERIES
ASMFLAGS += -DNRF_LOG_USES_RTT=1
ASMFLAGS += -DNRF_MESH_LOG_ENABLE=NRF_LOG_USES_RTT
ASMFLAGS += -DCONFIG_APP_IN_CORE
ASMFLAGS += -DUSE_APP_CONFIG
#ASMFLAGS += -DMESH_FEATURE_GATT=1
#ASMFLAGS += -DGATT_PROXY=1

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52832_xxaa: CFLAGS += -D__HEAP_SIZE=1024
nrf52832_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__HEAP_SIZE=1024
nrf52832_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52832_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_6.0.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_6.0.0_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ../config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
