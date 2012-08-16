/** ============================================================================
 *  @file   _idm_usr.h
 *
 *  @path   $(APUDRV)/gpp/src/api/
 *
 *  @desc   Declarations of ID manager control structures and definitions
 *          of ID management functions on the user-side.
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


#if !defined (_IDM_USR_H)
#define _IDM_USR_H


/*  ----------------------------------- APU Driver               */
#include <apudrv.h>


#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
 *  @name   IDM_Attrs
 *
 *  @desc   Attributes for creation of the IDM object
 *
 *  @field  baseId
 *              Base ID supported by this IDM object.
 *  @field  maxIds
 *              Maximum number of IDs supported by this IDM objects.
 *  ============================================================================
 */
typedef struct IDM_Attrs_tag {
    Uint16  baseId ;
    Uint16  maxIds ;
} IDM_Attrs ;


/** ============================================================================
 *  @func   _IDM_USR_init
 *
 *  @desc   Initializes the IDM component
 *
 *  @arg    None
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @enter  None.
 *
 *  @leave  IDM component is initialized
 *
 *  @see    IDM_exit ()
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_init (Void) ;


/** ============================================================================
 *  @func   _IDM_USR_exit
 *
 *  @desc   Finalizes the IDM component
 *
 *  @arg    None
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @leave  IDM component must be initialized
 *
 *  @leave  IDM component is finalized
 *
 *  @see    IDM_init ()
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_exit (Void) ;


/** ============================================================================
 *  @func   _IDM_USR_create
 *
 *  @desc   Creates an IDM object identified based on a unique key specified by
 *          the user.
 *
 *  @arg    key
 *              Unique key used to identify the IDM object created.
 *  @arg    attrs
 *              Attributes for creation of the IDM object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EINVALIDARG
 *              Invalid argument.
 *          DSP_ERESOURCE
 *              All IDM objects have been used.
 *          DSP_EMEMORY
 *              Operation failed due to memory error.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @leave  IDM component must be initialized
 *          attrs must be valid.
 *
 *  @leave  None
 *
 *  @see    IDM_delete ()
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_create (IN Uint32 key, IN IDM_Attrs * attrs) ;


/** ============================================================================
 *  @func   _IDM_USR_delete
 *
 *  @desc   Deletes an IDM object identified based on a unique key specified by
 *          the user.
 *
 *  @arg    key
 *              Unique key used to identify the IDM object being deleted.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EINVALIDARG
 *              Invalid argument.
 *          DSP_ENOTFOUND
 *              Object corresponding to specified key not found.
 *          DSP_EMEMORY
 *              Operation failed due to memory error.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @leave  IDM component must be initialized
 *
 *  @leave  None
 *
 *  @see    LIST_Create
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_delete (IN Uint32 key) ;


/** ============================================================================
 *  @func   _IDM_USR_acquireId
 *
 *  @desc   Acquires a free ID for the specified IDM object.
 *
 *  @arg    key
 *              Unique key used to identify the IDM object.
 *  @arg    idKey
 *              String key to associate with the ID to be returned. If the
 *              specified idKey already exists for the IDM object, the id for
 *              this idKey is returned and a reference count incremented. If the
 *              idKey does not exist, a new id is acquired and returned for this
 *              idKey.
 *  @arg    id
 *              Location to receive the ID being acquired.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_SEXISTS
 *              The specified idKey already exists and its ID is returned.
 *          DSP_EINVALIDARG
 *              Invalid argument.
 *          DSP_ENOTFOUND
 *              Object corresponding to specified key not found.
 *          DSP_ERESOURCE
 *              All IDs for this object have been consumed.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @leave  IDM component must be initialized
 *          idKey must be valid.
 *          id must be a valid pointer.
 *
 *  @leave  None
 *
 *  @see    IDM_releaseId ()
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_acquireId (IN Uint32 key, IN Pstr idKey, OUT Uint32 * id) ;


/** ============================================================================
 *  @func   _IDM_USR_releaseId
 *
 *  @desc   Releases the specified ID for the specified IDM object. The
 *          reference count for the id is decremented, and the id is released
 *          only if the count reaches zero.
 *
 *  @arg    key
 *              Unique key used to identify the IDM object.
 *  @arg    id
 *              ID to be released.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_SFREE
 *              The last release for specified ID resulted in it getting freed.
 *          DSP_EINVALIDARG
 *              Invalid argument.
 *          DSP_ENOTFOUND
 *              Object corresponding to specified key not found.
 *          DSP_EFAIL
 *              General Failure.
 *
 *  @leave  IDM component must be initialized
 *
 *  @leave  None
 *
 *  @see    IDM_acquireId ()
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
_IDM_USR_releaseId (IN Uint32 key, IN Uint32 id) ;


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* !defined (_IDM_USR_H) */

