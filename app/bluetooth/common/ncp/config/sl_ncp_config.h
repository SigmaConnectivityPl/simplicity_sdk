/***************************************************************************//**
 * @file
 * @brief Bluetooth Network Co-Processor (NCP) Interface Configuration
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

#ifndef SL_NCP_CONFIG_H
#define SL_NCP_CONFIG_H

/***********************************************************************************************//**
 * @addtogroup ncp
 * @{
 **************************************************************************************************/

// <<< Use Configuration Wizard in Context Menu >>>

// <h> General settings

// <o SL_NCP_CMD_BUF_SIZE> Command buffer size (bytes) <260-1024>
// <i> Default: 260
// <i> Define the size of Bluetooth NCP command buffer in bytes.
#define SL_NCP_CMD_BUF_SIZE     (260)

// <o SL_NCP_EVT_BUF_SIZE> Event buffer size (bytes) <260-4096>
// <i> Default: 260
// <i> Define the size of Bluetooth NCP event buffer in bytes.
#define SL_NCP_EVT_BUF_SIZE     (260)

// <o SL_NCP_CMD_TIMEOUT_MS> Command timeout (ms) <0-10000>
// <i> Default: 500
// <i> Allowed timeout in ms for command reception before triggering error.
#define SL_NCP_CMD_TIMEOUT_MS   (500)

// </h>

// <h> Debug settings

// <q SL_NCP_EMIT_SYSTEM_ERROR_EVT> System error event on incomplete command reception
// <i> Enable sending of a system error event with SL_STATUS_COMMAND_INCOMPLETE status on incomplete command reception.
// <i> The system error data may contain full or partial BGAPI message header data for analysis by the application.
// <i> Default: off
#define SL_NCP_EMIT_SYSTEM_ERROR_EVT  0

// </h>

// <<< end of configuration section >>>

/** @} (end addtogroup ncp) */
#endif // SL_NCP_CONFIG_H
