/** ============================================================================
 *  @file   zsp800m_phy_shmem.h
 *
 *  @path   $(APUDRV)/gpp/src/inc/sys/arch/ZSP800M/Linux/
 *
 *  @desc   Physical Interface Abstraction Layer for ZSP800M.
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


#if !defined (ZSP800M_PHY_SHMEM_H)
#define ZSP800M_PHY_SHMEM_H


/*  ----------------------------------- APU Driver               */
#include <apudrv.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <hal.h>


#if defined (__cplusplus)
extern "C" {
#endif

/*  ============================================================================
 *  @const  GENERAL_CONTROL_BASE/IVA2_CM_BASE/CORE_CM_BASE/PER_CM_BASE
 *          MAILBOX_BASE/MMU_BASE
 *
 *  @desc   Base address of different Peripherals.
 *  ============================================================================
 */
#define GENERAL_CONTROL_BASE   0x48002270
#define IVA2_CM_BASE           0x48004000
#define CORE_CM_BASE           0x48004A00
#define PER_CM_BASE            0x48005000
#define IVA2_PRM_BASE          0x48306000
#define MAILBOX_BASE           0x48094000
#define MMU_BASE               0x5D000000
#define MMU_SIZE                   0x1000

/*  ============================================================================
*  @const  SCM_BASE
*
*  @desc  base address of  system control module base.
*  ============================================================================
*/
#if PLATFORM_PBX
#define SCM_BASE            (0xC0001000L)
#else
#define SCM_BASE            (0X05822000L)
#endif

/*  ============================================================================
*  @const  PRCM_BASE
*
*  @desc   base address of  Power Reset and Clock Module
*  ============================================================================
*/
#if PLATFORM_PBX
#define PRCM_BASE            (0xC0001000L) 
#else
#define PRCM_BASE            (0X05821000L)
#endif

/*  ============================================================================
*  @const  IMEM_BASE
*
*  @desc   base address of instruction memory of ZSP
*  ============================================================================
*/
#if PLATFORM_PBX
#define IMEM_BASE            (0xc7040000L) 
#else
#define IMEM_BASE            (0x07040000L)
#endif

/*  ============================================================================
*  @const  DMEM_BASE
*
*  @desc   base address of data memory of ZSP
*  ============================================================================
*/
#if PLATFORM_PBX
#define DMEM_BASE            (0xc7000000L) 
#else
#define DMEM_BASE            (0x07000000L)
#endif

/*  ============================================================================
*  @const  IMEM_BASE
*
*  @desc   base address of instruction memory of ZSP
*  ============================================================================
*/
#define IMEM_SIZE            (0x10000)


/*  ============================================================================
*  @const  DMEM_BASE
*
*  @desc   base address of data memory of ZSP
*  ============================================================================
*/
#define DMEM_SIZE            (0x20000)


/** ============================================================================
 *  @name   ZSP800M_shmemInterface
 *
 *  @desc   Interface functions exported by the Shared Driver subcomponent.
 *  ============================================================================
 */
extern HAL_Interface ZSP800M_shmemInterface ;


/** ============================================================================
 *  @func   ZSP800M_phyShmemInit
 *
 *  @desc   Initializes Shared Driver/device.
 *
 *  @arg    halObject.
 *              HAL object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              All other error conditions.
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
ZSP800M_phyShmemInit (IN Pvoid halObj) ;


/** ============================================================================
 *  @func   ZSP800M_phyShmemExit
 *
 *  @desc   Finalizes Shared Driver/device.
 *
 *  @arg    halObject.
 *              HAL object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              All other error conditions.
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
ZSP800M_phyShmemExit (IN Pvoid halObj) ;


#if defined (__cplusplus)
}
#endif


#endif  /* !defined (ZSP800M_PHY_SHMEM_H) */
