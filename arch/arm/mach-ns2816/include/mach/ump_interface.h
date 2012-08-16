/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file ump_kernel_interface.h
 *
 * This file contains some kernel interfaces of the UMP API.
 */

#ifndef __UMP_INTERFACE_DECLARE_H__
#define __UMP_INTERFACE_DECLARE_H__


/** @defgroup ump_kernel_space_api UMP Kernel Space API
 * @{ */



#ifdef __cplusplus
extern "C"
{
#endif


/**
 * External representation of a UMP handle in kernel space.
 */
typedef void * ump_dd_handle;

/**
 * Typedef for a secure ID, a system wide identificator for UMP memory buffers.
 */
typedef unsigned int ump_secure_id;

/**
 * Struct used to describe a physical block used by UMP memory
 */
typedef struct ump_dd_physical_block
{
	unsigned long addr; /**< The physical address of the block */
	unsigned long size; /**< The length of the block, typically page aligned */
} ump_dd_physical_block;

ump_secure_id ump_dd_secure_id_get(ump_dd_handle memh);
ump_dd_handle ump_dd_handle_create_from_phys_blocks(ump_dd_physical_block * blocks, unsigned long num_blocks);

#ifdef __cplusplus
}
#endif


/** @} */ /* end group ump_kernel_space_api */


#endif  /* __UMP_INTERFACE_DECLARE_H__ */
