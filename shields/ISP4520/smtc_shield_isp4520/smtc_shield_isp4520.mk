# --- The Clear BSD License ---
# Copyright Semtech Corporation 2022. All rights reserved.
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

define SMTC_SHIELD_ISP4520_SUPPORTED_BODY
Shield $(SMTC_SHIELD_ISP4520) is not supported.

The supported shields are:
  * SX1261: ISP4520_EU / ISP4520_AS
  * SX1262: ISP4520_US
endef

C_INCLUDES += \
-I$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/common/inc \

C_SOURCES += \
$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/common/src/smtc_shield_isp4520_ant_sw.c \
$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/common/src/smtc_shield_isp4520_led.c \

ifeq ($(SMTC_SHIELD_ISP4520), ISP4520_EU)
C_SOURCES += $(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_EU/src/smtc_shield_isp4520_eu.c
C_INCLUDES += -I$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_EU/inc

else ifeq ($(SMTC_SHIELD_ISP4520), ISP4520_AS)
C_SOURCES += $(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_AS/src/smtc_shield_isp4520_as.c
C_INCLUDES += -I$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_AS/inc

else ifeq ($(SMTC_SHIELD_ISP4520), ISP4520_US)
C_SOURCES += $(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_US/src/smtc_shield_isp4520_us.c
C_INCLUDES += -I$(SMTC_SHIELD_ISP4520_DIR)/smtc_shield_isp4520/ISP4520_US/inc

else
$(error $(SMTC_SHIELD_ISP4520_SUPPORTED_BODY))
endif
