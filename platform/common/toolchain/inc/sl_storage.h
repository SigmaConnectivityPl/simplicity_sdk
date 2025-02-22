/***************************************************************************//**
 * @file
 * @brief Flash storage reservation api
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#ifndef SL_STORAGE_H
#define SL_STORAGE_H

/***************************************************************************//**
 * @addtogroup linker
 * @{
 ******************************************************************************/

#include "sl_memory_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief
 *   Get size and location of the bootloader storage region.
 *
 * @return
 *   description of the region reserved for bootloader storage.
 ******************************************************************************/
sl_memory_region_t sl_storage_get_bootloader_region(void);

/** @} end linker */

#ifdef __cplusplus
}
#endif

#endif // SL_STORAGE_H
