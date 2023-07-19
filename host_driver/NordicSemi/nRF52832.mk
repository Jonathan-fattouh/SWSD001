# --- The Clear BSD License ---
# Copyright Semtech Corporation 2021. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Semtech corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

######################################
# source
######################################

# C sources

C_SOURCES +=  \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/fprintf/nrf_fprintf.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/fprintf/nrf_fprintf_format.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/segger_rtt/SEGGER_RTT.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/segger_rtt/SEGGER_RTT_printf.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_frontend.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_str_formatter.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_default_backends.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_backend_uart.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_backend_serial.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src/nrf_log_backend_rtt.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util/app_error.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util/app_error_handler_gcc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util/app_error_weak.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util/app_util_platform.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util/nrf_assert.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/atomic_fifo/nrf_atfifo.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/atomic/nrf_atomic.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/balloc/nrf_balloc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/fstorage/nrf_fstorage.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/fstorage/nrf_fstorage_nvmc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/memobj/nrf_memobj.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/queue/nrf_queue.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/ringbuf/nrf_ringbuf.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/experimental_section_vars/nrf_section_iter.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/sortlist/nrf_sortlist.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/strerror/nrf_strerror.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/soc/nrfx_atomic.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_adc.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_clock.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_comp.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_dppi.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_gpiote.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_lpcomp.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_nfct.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_nvmc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_pdm.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_power.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_ppi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_pwm.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_qdec.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_qspi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_rng.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_rtc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_saadc.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_spi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_spim.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_spis.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_swi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_systick.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_temp.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_timer.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_twi_twim.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_twim.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_twis.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_uart.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_uarte.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/nrfx_wdt.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/src/prs/nrfx_prs.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/hal/nrf_nvmc.c\
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_clock.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_power.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_rng.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_spi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_twi.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy/nrf_drv_uart.c \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/mdk/system_nrf52.c \
$(TOP_DIR)/host_driver/NordicSemi/custom_lib/timer/app_timer2.c  \
$(TOP_DIR)/host_driver/NordicSemi/custom_lib/timer/drv_rtc.c

ASM_SOURCES +=  \
$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/mdk/gcc_startup_nrf52.s

#######################################
# CFLAGS
#######################################

CPU = -mcpu=cortex-m4

FPU = -mfpu=fpv4-sp-d16

FLOAT-ABI = -mfloat-abi=hard

SEP_SECTIONS = -ffunction-sections -fdata-sections -fno-strict-aliasing

MCU = $(CPU) -mthumb -mabi=aapcs $(FPU) $(FLOAT-ABI) $(SEP_SECTIONS) --specs=nano.specs -g3

C_DEFS +=  \
-DAPP_TIMER_V2 \
-DAPP_TIMER_V2_RTC1_ENABLED \
-DFLOAT_ABI_HARD \
-DNRF52 \
-DNRF52832_XXAA \
-DNRF52_PAN_74 \
-DCONFIG_GPIO_AS_PINRESET \
-D__HEAP_SIZE=8192 \
-D__STACK_SIZE=8192

C_INCLUDES +=  \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/toolchain/cmsis/include \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/drivers_nrf/nrf_soc_nosd \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/atomic \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/atomic_fifo \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/balloc \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/button \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/delay \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/experimental_section_vars \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/fds \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/fifo \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/fstorage \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/gpiote \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/log/src \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/memobj \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/queue \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/pwr_mgmt \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/ringbuf \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/scheduler \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/sortlist \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/strerror \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/timer \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/util \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/components/libraries/mutex \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/fprintf \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/external/segger_rtt \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/integration/nrfx/legacy \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx  \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/drivers/include  \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/hal  \
-I$(TOP_DIR)/host_driver/NordicSemi/nRF5_SDK_17.0.2_d674dde/modules/nrfx/mdk  \
-I$(TOP_DIR)/host_driver/NordicSemi/custom_lib/timer  \
-I$(TOP_DIR)/host_driver/NordicSemi/config

# Determine the linker script to use based on TARGET_MCU
ifeq ($(TARGET_MCU), nRF52832)
LDSCRIPT = $(TOP_DIR)/host_driver/NordicSemi/gcc/gcc_nRF52.ld
else
$(error Invalid target, must be nRF52832 or please modify makefile to add right .ld file)
endif
