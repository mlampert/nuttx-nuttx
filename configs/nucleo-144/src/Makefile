############################################################################
# configs/nucleo-144/src/Makefile
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#            David Sidrane <david_s5@nscdg.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = stm32_boot.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_SPI),y)
CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_MMCSD),y)
ifeq ($(CONFIG_ARCH_CHIP_STM32F767ZI),y)
CSRCS += stm32_mmcsd.c
else
CSRCS += stm32_sdio.c
endif
endif

ifeq ($(CONFIG_STM32F7_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_STM32F7_BBSRAM),y)
CSRCS += stm32_bbsram.c
endif

ifeq ($(CONFIG_LCD_SSD1306_I2C),y)
CSRCS += stm32_ssd1306.c
endif

ifeq ($(CONFIG_SENSORS_VL53L0X),y)
CSRCS += stm32_vl53l0x.c
endif

include $(TOPDIR)/configs/Board.mk
