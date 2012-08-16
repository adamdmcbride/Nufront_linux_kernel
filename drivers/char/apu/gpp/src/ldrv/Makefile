#   ============================================================================
#   @file   Makefile
#
#   @path   $(APUDRV)/gpp/src/ldrv/
#
#   @desc   This file is a standard interface to the make scripts.
#           Usually no change is required in this file.
#
#   @ver    0.01.00.00
#   ============================================================================
#   Copyright (C) 2011-2012, Nufront Incorporated - http://www.nufront.com/
#
#   This program is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation version 2.
#   
#   This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
#   whether express or implied; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#   General Public License for more details.
#   ============================================================================


#   ============================================================================
#   Set the device type (GPP/DSP)
#   ============================================================================

export DEVICETYPE := GPP


#   ============================================================================
#   Get the directory separator used on the development host.
#   ============================================================================

ifneq ("$(ComSpec)", "")
    ifeq ("$(OSTYPE)", "cygwin")
        DIRSEP ?=/
    else
        DIRSEP ?=\\
    endif
else
    DIRSEP ?= /
endif


#   ============================================================================
#   Start the build process
#   ============================================================================

include $(APUDRV)$(DIRSEP)make$(DIRSEP)start.mk
