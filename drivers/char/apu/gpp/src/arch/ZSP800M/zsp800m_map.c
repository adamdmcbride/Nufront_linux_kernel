/** ============================================================================
 *  @file   zsp800m_map.c
 *
 *  @path   $(APUDRV)/gpp/src/arch/ZSP800M/
 *
 *  @desc   Defines the configuration mapping information for the APU DRIVER
 *          driver.
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


/*  ----------------------------------- APU DRIVER Headers       */
#include <apudrv.h>
#include <_apudrv.h>

#if defined (POOL_COMPONENT)
#include <pooldefs.h>
#endif /* if defined (POOL_COMPONENT) */

#if defined (MSGQ_COMPONENT)
#include <msgqdefs.h>
#endif /* if defined (MSGQ_COMPONENT) */

#include <cfgmap.h>

#if defined (__cplusplus)
EXTERN "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @name   ZSP800MMAP_DspObjects
 *
 *  @desc   Array of configuration mapping objects for the DSPs in the system.
 *  ============================================================================
 */
EXTERN DSP_Interface    ZSP800M_Interface ;
CONST CFGMAP_Dsp  ZSP800MMAP_DspObjects [] =
{
    {
        "ZSP800M",                  /* NAME           : Name of the DSP */
        &ZSP800M_Interface          /* INTERFACE      : DSP interface table */
    }
} ;

/** ============================================================================
 *  @name   ZSP800MMAP_LoaderObjects
 *
 *  @desc   Array of configuration mapping objects for the DSP executable
 *          loaders in the system.
 *  ============================================================================
 */
EXTERN KFILE_Interface KFILEPSEUDO_Interface ;
EXTERN KFILE_Interface KFILEDEF_Interface ;
CONST CFGMAP_Loader  ZSP800MMAP_LoaderObjects [] =
{
    {
		"BIN",
		NULL,//&BINFILE_Interface,
		&KFILEDEF_Interface
	}
} ;

#if (!defined (ONLY_PROC_COMPONENT))
/** ============================================================================
 *  @name   ZSP800MMAP_LinkDrvObjects
 *
 *  @desc   Array of configuration mapping objects for the link drivers in the
 *          system.
 *  ============================================================================
 */
EXTERN DRV_Interface SHMDRV_Interface ;
CONST CFGMAP_LinkDrv  ZSP800MMAP_LinkDrvObjects [] =
{
    {
        "SHMDRV",                      /* NAME           : Name of the link driver */
        &SHMDRV_Interface              /* INTERFACE      : Link driver interface table */
    }
} ;

/** ============================================================================
 *  @name   ZSP800MMAP_IpsObjects
 *
 *  @desc   Array of configuration mapping objects for the IPS components in the
 *          system.
 *  ============================================================================
 */
EXTERN FnIpsInit       IPS_init ;
EXTERN FnIpsExit       IPS_exit ;
#if defined (DDSP_DEBUG)
EXTERN FnIpsDebug      IPS_debug ;
#endif /* if defined (DDSP_DEBUG) */
CONST CFGMAP_Ips  ZSP800MMAP_IpsObjects [] =
{
    {
        "IPS",                         /* NAME           : Name of the IPS */
        (FnIpsInit)  &IPS_init,        /* FXN_INIT       : Init function for the IPS */
        (FnIpsExit)  &IPS_exit,        /* FXN_EXIT       : Exit function for the IPS */
#if defined (DDSP_DEBUG)
        (FnIpsDebug) &IPS_debug        /* FXN_DEBUG      : Debug function for the IPS */
#endif /* if defined (DDSP_DEBUG) */
    }
} ;
#endif /* if (!defined (ONLY_PROC_COMPONENT)) */


#if defined (POOL_COMPONENT)
/** ============================================================================
 *  @name   ZSP800MMAP_PoolObjects
 *
 *  @desc   Array of configuration mapping objects for the POOLs in the system.
 *  ============================================================================
 */
EXTERN FnPoolInit       SMAPOOL_init ;
EXTERN FnPoolExit       SMAPOOL_exit ;
EXTERN POOL_Interface   SMAPOOL_Interface ;

#if defined (PCPY_LINK)
EXTERN FnPoolInit       BUFPOOL_init ;
EXTERN FnPoolExit       BUFPOOL_exit ;
EXTERN POOL_Interface   BUFPOOL_Interface ;
#endif /* if defined (PCPY_LINK) */
CONST CFGMAP_Pool  ZSP800MMAP_PoolObjects [] =
{
    {
        "SMAPOOL",                     /* NAME           : Name of the pool */
        (FnPoolInit) &SMAPOOL_init,    /* FXN_INIT       : Init function for the pool */
        (FnPoolExit) &SMAPOOL_exit,    /* FXN_EXIT       : Exit function for the pool */
        &SMAPOOL_Interface             /* INTERFACE      : Pool interface table */
    },
#if defined (PCPY_LINK)
    {
        "BUFPOOL",                     /* NAME           : Name of the pool */
        (FnPoolInit) &BUFPOOL_init,    /* FXN_INIT       : Init function for the pool */
        (FnPoolExit) &BUFPOOL_exit,    /* FXN_EXIT       : Exit function for the pool */
        &BUFPOOL_Interface             /* INTERFACE      : Pool interface table */
    }
#endif /* if defined (PCPY_LINK) */
} ;
#endif /* if defined (POOL_COMPONENT) */

#if defined (CHNL_COMPONENT)
/** ============================================================================
 *  @name   ZSP800MMAP_DataDrvObjects
 *
 *  @desc   Array of configuration mapping objects for the Data drivers in the
 *          system.
 *  ============================================================================
 */
EXTERN DATA_Interface ZCPYDATA_Interface ;
CONST CFGMAP_DataDrv  ZSP800MMAP_DataDrvObjects [] =
{
    {
        "ZCPYDATA",                /* NAME           : Name of the data driver */
        &ZCPYDATA_Interface        /* INTERFACE      : Data transfer interface table */
    }
} ;
#endif /* if defined (CHNL_COMPONENT) */

#if defined (MSGQ_COMPONENT)
/** ============================================================================
 *  @name   ZSP800MMAP_MqtObjects
 *
 *  @desc   Array of configuration mapping objects for the Message Queue
 *          Transports in the system.
 *  ============================================================================
 */
EXTERN MQT_Interface ZCPYMQT_Interface ;
CONST CFGMAP_Mqt  ZSP800MMAP_MqtObjects [] =
{
    {
        "ZCPYMQT",                     /* NAME           : Name of the Message Queue Transport */
        &ZCPYMQT_Interface             /* INTERFACE      : MQT Interface table */
    }
} ;

#endif /* if defined (MSGQ_COMPONENT) */

/** ============================================================================
 *  @name   ZSP800MMAP_Config
 *
 *  @desc   APU DRIVER configuration mapping structure.
 *  ============================================================================
 */
CFGMAP_Object  ZSP800MMAP_Config = {
    1,                                              /* NUMDSPS        : Number of types of DSPs */
    (CFGMAP_Dsp *)     ZSP800MMAP_DspObjects,     /* DSPOBJECTS     : Array of DSP configuration mapping objects */
    1,                                              /* NUMLOADERS     : Number of types of DSP executable loaders */
    (CFGMAP_Loader *)  ZSP800MMAP_LoaderObjects,  /* LOADERS        : Array of DSP executable loader configuration mapping objects */

#if (!defined (ONLY_PROC_COMPONENT))
    1,                                              /* NUMLINKDRVS    : Number of types of link drivers */
    (CFGMAP_LinkDrv *) ZSP800MMAP_LinkDrvObjects, /* LINKDRVOBJECTS : Array of Link Driver configuration mapping objects */
    1,                                              /* NUMIPS         : Number of types of IPS */
    (CFGMAP_Ips *)     ZSP800MMAP_IpsObjects,     /* IPSOBJECTS     : Array of IPS configuration mapping objects */
#else
    0,                                              /* NUMLINKDRVS    : Number of types of link drivers */
    NULL,                                           /* LINKDRVOBJECTS : Array of Link Driver configuration mapping objects */
    0,                                              /* NUMIPS         : Number of types of IPS */
    NULL,                                           /* IPSOBJECTS     : Array of IPS configuration mapping objects */
#endif /* if (!defined (ONLY_PROC_COMPONENT)) */

#if defined (POOL_COMPONENT)
    1,                                              /* NUMPOOLS       : Number of types of POOLs */
    (CFGMAP_Pool *)    ZSP800MMAP_PoolObjects,    /* POOLOBJECTS    : Array of POOL configuration mapping objects */
#endif /* if defined (POOL_COMPONENT) */
#if defined (CHNL_COMPONENT)
    1,                                              /* NUMDATADRIVERS : Number of types of Data drivers */
    (CFGMAP_DataDrv *) ZSP800MMAP_DataDrvObjects, /* DATADRIVERS    : Array of Data driver configuration mapping objects */
#endif /* if defined (CHNL_COMPONENT) */
#if defined (MSGQ_COMPONENT)
    1,                                              /* NUMMQTS        : Number of types of MQTs */
    (CFGMAP_Mqt *)     ZSP800MMAP_MqtObjects      /* MQTOBJECTS     : Array of MQT configuration mapping objects */
#endif /* if defined (MSGQ_COMPONENT) */
} ;


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


