#   =========================================================
#   DSP/BIOS LINK Configuration file.
#
#   CAUTION! This is a generated file.
#            All changes will be lost.
#
#   This file was generated on MAR 6, 2012  1:33:20
#   =========================================================


#   =========================================================
#   When this file was created.
#   =========================================================
export  NU_APUDRV_CFGDATE            := MAR 6, 2012  1:33:20


#   =========================================================
#   DSP/BIOS LINK GPP side root directory.
#   =========================================================
export NU_APUDRV_GPPROOT             := ${APUDRV}/gpp


#   =========================================================
#   GPP OS for which DSP/BIOS LINK is being built.
#   =========================================================
export NU_APUDRV_GPPOS               := Linux
export NU_APUDRV_GPPOSPREFIX         := 
export NU_APUDRV_GPPDEVICE           := ARM
export NU_APUDRV_GPPOSVERSION        := 3.0.0
export NU_APUDRV_GPPDISTRIBUTION     := nusmart_3.0


#   =========================================================
#   Target platform for DSP/BIOS LINK.
#   =========================================================
export NU_APUDRV_PLATFORM            := NUSMART


#   =========================================================
#   DSPs for which DSP/BIOS LINK is being built.
#   =========================================================
#export NU_APUDRV_DSPDEVICES          := DM6467GEM
export NU_APUDRV_DSPDEVICES          := ZSP800M


#   =========================================================
#   External DSPS name for which DSP/BIOS LINK is being built.
#   =========================================================
export NU_APUDRV_DSPDEVICES_EXTERNAL := 


#   =========================================================
#   DSP Config files.
#   =========================================================
#export NU_APUDRV_DSPCFGFILES        := CFG_DM6467GEM_SHMEM.c 
export NU_APUDRV_DSPCFGFILES        := CFG_ZSP800M_SHMEM.c 


#   =========================================================
#   Processor Counts and IDs.
#   =========================================================
export NU_APUDRV_MAX_PROCESSORS     := 2
export NU_APUDRV_MAX_DSPS           := 1
export NU_APUDRV_ID_GPP             := 1


#   =========================================================
#   Default DSP Device and its physical interfaces for DSP/BIOS LINK.
#   =========================================================
ifndef NU_APUDRV_BUILD_DEFAULTDSP
#export  NU_APUDRV_DSPDEVICE         := DM6467GEM
export  NU_APUDRV_DSPDEVICE         := ZSP800M
export  NU_APUDRV_DSPDEVICE_EXTERNAL:= 
export  NU_APUDRV_PROCID            := 0
export  NU_APUDRV_DSPTEMPPATH       := 
endif


#   =========================================================
#   Target DSP Devices and their physical interfaces for DSP/BIOS LINK.
#   =========================================================
#export NU_APUDRV_DM6467GEM_PHYINTERFACE        := SHMEM
export NU_APUDRV_ZSP800M_PHYINTERFACE          := SHMEM
export NU_APUDRV_ZSP800M_PROCID              := 0


#   =========================================================
#   Compile time scalability options for DSP/BIOS LINK.
#   =========================================================
export NU_APUDRV_USE_PROC            := 1
export NU_APUDRV_USE_POOL            := 1
export NU_APUDRV_USE_NOTIFY          := 0
export NU_APUDRV_USE_MPCS            := 1
export NU_APUDRV_USE_RINGIO          := 0
export NU_APUDRV_USE_MPLIST          := 1
export NU_APUDRV_USE_MSGQ            := 1
export NU_APUDRV_USE_CHNL            := 0


#   =========================================================
#   Compile time physical link scalability options for device.
#   =========================================================
export NU_APUDRV_USE_MSGQ_ZCPY_LINK  := 1
export NU_APUDRV_USE_CHNL_ZCPY_LINK  := 0


#   =========================================================
#   Enable/ Disable Trace.
#   =========================================================
export NU_APUDRV_TRACE  := 0


#   =========================================================
#   GPP INC PATH
#   =========================================================
export  NU_APUDRV_GPP_INC_PATH := ${APUDRV}/gpp/inc ${APUDRV}/gpp/inc/usr ${APUDRV}/gpp/inc/sys ${APUDRV}/gpp/inc/sys/arch ${APUDRV}/gpp/inc/sys/arch/ZSP800M ${APUDRV}/gpp/inc/sys/arch/ZSP800M/Linux ${APUDRV}/gpp/inc/sys/Linux ${APUDRV}/gpp/inc/sys/Linux/3.0.0 ${APUDRV}/gpp/src/gen ${APUDRV}/gpp/src/gen/Linux ${APUDRV}/gpp/src/gen/Linux/3.0.0 ${APUDRV}/gpp/src/osal ${APUDRV}/gpp/src/osal/Linux ${APUDRV}/gpp/src/osal/Linux/3.0.0 ${APUDRV}/gpp/src/pmgr ${APUDRV}/gpp/src/pmgr/Linux ${APUDRV}/gpp/src/pmgr/Linux/3.0.0 ${APUDRV}/gpp/src/api ${APUDRV}/gpp/src/api/Linux ${APUDRV}/gpp/src/api/Linux/3.0.0 ${APUDRV}/gpp/src/ldrv ${APUDRV}/gpp/src/ldrv/Linux ${APUDRV}/gpp/src/ldrv/Linux/3.0.0 ${APUDRV}/gpp/src/ldrv/DRV ${APUDRV}/gpp/src/ldrv/DATA ${APUDRV}/gpp/src/ldrv/MPCS ${APUDRV}/gpp/src/ldrv/MPLIST ${APUDRV}/gpp/src/ldrv/MQT ${APUDRV}/gpp/src/ldrv/IPS ${APUDRV}/gpp/src/ldrv/SMM ${APUDRV}/gpp/src/ldrv/RINGIO ${APUDRV}/gpp/src/ldrv/POOLS



#   =========================================================
#   GPP DEFINES
#   =========================================================
export  NU_APUDRV_GPP_DEFINES :=  OS_LINUX MAX_DSPS=1 MAX_PROCESSORS=2 ID_GPP=1 PROC_COMPONENT POOL_COMPONENT MPCS_COMPONENT MPLIST_COMPONENT MSGQ_COMPONENT MSGQ_ZCPY_LINK ZCPY_LINK KFILE_DEFAULT  ZSP800M ZSP800M_PHYINTERFACE=SHMEM_INTERFACE GPP_SWI_MODE PLATFORM_PBX=0

