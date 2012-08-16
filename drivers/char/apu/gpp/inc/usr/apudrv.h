/** ============================================================================
 *  @file   apudrv.h
 *
 *  @path   $(APUDRV)/gpp/inc/usr/
 *
 *  @desc   Defines data types and structures used by APU Driver.
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



#if !defined (APUDRV_H)
#define APUDRV_H


/*  ----------------------------------- APU Driver                   */
#include <gpptypes.h>
#include <constants.h>
#include <errbase.h>
#include <archdefs.h>
#include <linkcfgdefs.h>

#if defined(ANDROID)
#include <dbdefs.h>
#endif

#if defined (__cplusplus)
extern "C" {
#endif

/*  ============================================================================
 *  @const  MAX_IPS
 *
 *  @desc   Maximum number of IPS objects supported for each DSP.
 *
 *  ============================================================================
 */
#define MAX_IPS             16u

/** ============================================================================
 *  @const  WAIT_FOREVER
 *
 *  @desc   Wait indefinitely.
 *  ============================================================================
 */
#define WAIT_FOREVER           (~((Uint32) 0u))

/** ============================================================================
 *  @const  WAIT_NONE
 *
 *  @desc   Do not wait.
 *  ============================================================================
 */
#define WAIT_NONE              ((Uint32) 0u)


/** ============================================================================
 *  @macro  IS_GPPID
 *
 *  @desc   Is the GPP ID valid.
 *  ============================================================================
 */
#define IS_GPPID(id)        (id == ID_GPP)


#if defined (__cplusplus)
}
#endif


#endif /* !defined (APUDRV_H) */
