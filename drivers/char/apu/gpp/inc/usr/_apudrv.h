/** ============================================================================
 *  @file   _apudrv.h
 *
 *  @path   $(APUDRV)/gpp/inc/usr/
 *
 *  @desc   Consolidate Include file to include all internal generic definition
 *          include files.
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



#if !defined (_APUDRV_H)
#define _APUDRV_H


#include <_bitops.h>
#include <_dspdefs.h>
#include <_linkdefs.h>
#include <_safe.h>
#include <_intobject.h>
#include <loaderdefs.h>

#include <_loaderdefs.h>

#if defined (POOL_COMPONENT)
#include <_pooldefs.h>
#endif

#if defined (CHNL_COMPONENT)
#include <_datadefs.h>
#endif

#endif /* if !defined (_APUDRV_H) */
