/** ============================================================================
 *  @file   zsp800m_hal_boot.h
 *
 *  @path   $(APUDRV)/gpp/inc/sys/arch/ZSP800M/
 *
 *  @desc   Hardware Abstraction Layer for Davinci Boot controller.
 *          Defines the Platform specific HAL (Hardware Abstraction Layer)
 *          object.
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



#if !defined (ZSP800M_HAL_BOOT_H)
#define ZSP800M_HAL_BOOT_H


/*  ----------------------------------- APU Driver               */
#include <apudrv.h>


#if defined (__cplusplus)
extern "C" {
#endif

/** ============================================================================
 *  @func   ZSP800M_halBootCtrl
 *
 *  @desc   Boot controller.
 *
 *  @arg    halObj.
 *              Pointer to HAL object.
 *  @arg    cmd.
 *              Command.
 *  @arg    arg.
 *              Command specific arguments.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EINVALIDARG
 *              Unsupported interrupt control command.
 *
 *  @enter  None.
 *
 *  @leave  None.
 *
 *  @see    None
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halBootCtrl (IN         Pvoid           halObj,
                     IN         DSP_BootCtrlCmd cmd,
                     IN OUT     Pvoid           arg) ;

#if defined (__cplusplus)
}
#endif


#endif  /* !defined (ZSP800M_HAL_BOOT_H) */


