/** ============================================================================
 *  @file   _pool.h
 *
 *  @path   $(APUDRV)/gpp/src/api/
 *
 *  @desc   Internal declarations for POOL component.
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


#if !defined (_POOL_H)
#define _POOL_H


/*  ----------------------------------- APU DRIVER Headers       */
#include <apudrv.h>
#include <_pooldefs.h>


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @const  MAX_ADDR_TYPES
 *
 *  @desc   Maximum number of address types.
 *  ============================================================================
 */
#define MAX_ADDR_TYPES      4u


/** ============================================================================
 *  @name   POOL_AddrInfo
 *
 *  @desc   This structure defines the config attributes required for
 *          mapping/unmapping the buffers.
 *
 *  @field  isInit
 *              Pool is initialized or not.
 *  @field  addr
 *              Array of addresses containing the same address in different
 *              address spaces.
 *  @field  size
 *              Size of memory block in bytes.
 *  ============================================================================
 */
typedef struct POOL_AddrInfo_tag {
    Bool    isInit ;
    Uint32  addr [MAX_ADDR_TYPES] ;
    Uint32  size ;
} POOL_AddrInfo ;


/** ============================================================================
 *  @func   POOL_addrConfig
 *
 *  @desc   Pool driver specific configuration data.
 *
 *  @modif  object.
 *  ============================================================================
 */
extern POOL_AddrInfo POOL_addrConfig [MAX_DSPS][MAX_POOLENTRIES] ;


/** ============================================================================
 *  @func   _POOL_init
 *
 *  @desc   This function initializes the DRV POOL component.
 *
 *  @arg    None
 *
 *  @ret    None
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
EXPORT_API
Void
_POOL_init (Void) ;


/** ============================================================================
 *  @func   _POOL_exit
 *
 *  @desc   This function finalizes the DRV POOL component.
 *
 *  @arg    None
 *
 *  @ret    None
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
EXPORT_API
Void
_POOL_exit (Void) ;


/** ============================================================================
 *  @func   _POOL_xltBuf
 *
 *  @desc   This function translates the buffer address between two address
 *          spaces.
 *
 *  @arg    poolId
 *              Pool Identification number.
 *  @arg    bufPtr
 *              Pointer to the buffer. It contains the translated address on
 *              successful completion.
 *  @arg    xltFlag
 *              Flag to indicate whether user to kernel or reverse direction
 *              translation required.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              General failure.
 *          DSP_ERANGE
 *              Invalid address range.
 *
 *  @enter  poolId should be valid.
 *          bufPtr should be valid.
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_POOL_xltBuf (IN     PoolId           poolId,
              IN OUT Pvoid *          bufPtr,
              IN     POOL_AddrXltFlag xltFlag) ;


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* !defined (_POOL_H) */
