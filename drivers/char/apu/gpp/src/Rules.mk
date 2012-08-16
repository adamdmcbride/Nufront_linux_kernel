#   ============================================================================
#   @file   Rules.mk
#
#   @path   $(APUDRV)/gpp/src/
#
#   @desc   This file contains the configurable items for the kbuild based
#           makefile.
#
#   @ver    0.01.00.00
#   ============================================================================
#   Copyright (C) 2011-2012, Nufront Incorporated -
#   http://www.nufront.com/
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#   
#   *  Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#   
#   *  Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#   
#   *  Neither the name of Nufront Incorporated nor the names of
#      its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#   
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#   ============================================================================

# Update these macros to reflect your environment.
#
# KERNEL_DIR   = The Linux kernel source directory
# TOOL_PATH    = Path to the toolchain
# MAKE_OPTS    = Architecture-specific Make options
#

ifeq ("$(NU_APUDRV_GPPOS)", "Linux")
#KERNEL_DIR    := /home/andy/ti/dvsdk/git
#TOOL_PATH     := /opt/arm-2010q1/bin
#KERNEL_DIR    := /home/andy/kernel/linux-2.6-stable
KERNEL_DIR    := ../../../../../
TOOL_PATH     :=

saved_cross := $(shell cat $(KERNEL_DIR)/include/generated/kernel.cross 2> /dev/null)
CROSS_COMPILE := $(saved_cross)
export CROSS_COMPILE

PATH  := $(TOOL_PATH):$(PATH)
export PATH

endif # ifeq ("$(NU_APUDRV_GPPOS)", "Linux")
