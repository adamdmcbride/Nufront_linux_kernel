/** ============================================================================
 *  @file   ldrv_utils.h
 *
 *  @path   $(APUDRV)/gpp/src/ldrv/
 *
 *  @desc   Defines generic utilities and macros for LDRV component.
 *
 *  @ver    0.01.00.00
 *  ============================================================================
 *  Copyright (C) 2011-2012, Nufront Incorporated - http://www.nufront.com/
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation version 2.
 *  
 *  This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 *  whether express or implied; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *  ============================================================================
 */


#if !defined (LDRV_UTILS_H)
#define LDRV_UTILS_H


/*  ----------------------------------- APU DRIVER Headers       */
#include <apudrv.h>
#include <_apudrv.h>


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @macro  BYTE_TO_MADU
 *
 *  @desc   Converts a size or address value from byte to DSP MADU.
 *  ============================================================================
 */
#define BYTE_TO_MADU(value, dspMaduSize) ((Uint32) (value) / (dspMaduSize))

/** ============================================================================
 *  @macro  MADU_TO_BYTE
 *
 *  @desc   Converts a size or address value from DSP MADU to byte.
 *  ============================================================================
 */
#define MADU_TO_BYTE(value, dspMaduSize) ((Uint32) (value) * (dspMaduSize))

/** ============================================================================
 *  @macro  ADDR_GPP_TO_DSP
 *
 *  @desc   Translates an address from GPP address space to DSP address space.
 *  ============================================================================
 */
#define ADDR_GPP_TO_DSP(gppBaseAddr, dspBaseAddr, addr, dspMaduSize)           \
            (  BYTE_TO_MADU (((Uint32) (addr) - (gppBaseAddr)), (dspMaduSize)) \
             + (dspBaseAddr))

/** ============================================================================
 *  @macro  ADDR_DSP_TO_GPP
 *
 *  @desc   Translates an address from DSP address space to GPP address space.
 *  ============================================================================
 */
#define ADDR_DSP_TO_GPP(gppBaseAddr, dspBaseAddr, addr, dspMaduSize)           \
            (  MADU_TO_BYTE (((Uint32) (addr) - (dspBaseAddr)), (dspMaduSize)) \
             + (gppBaseAddr))


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif  /* !defined (LDRV_UTILS_H) */
