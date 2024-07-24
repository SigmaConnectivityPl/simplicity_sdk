/***************************************************************************//**
 * @file
 * @brief CS Initiator example configuration
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
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

#ifndef INITIATOR_APP_CONFIG_H
#define INITIATOR_APP_CONFIG_H

#include "cs_initiator_config.h"
#include "sl_bt_api.h"
#include "sl_rtl_clib_api.h"

// <<< Use Configuration Wizard in Context Menu >>>

// <o INITIATOR_APP_CONFIG_MEASUREMENT_MODE> Measurement mode.
// <i> Specify measurement mode.
// <sl_bt_cs_mode_rtt=> Round Trip Time
// <sl_bt_cs_mode_pbr=> Phase Based Ranging
// <i> Default: sl_bt_cs_mode_pbr
#define MEASUREMENT_MODE     sl_bt_cs_mode_pbr

// <o OBJECT_TRACKING_MODE> Specify object tracking mode.
// <i> Specify object tracking mode.
// <SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC=> Moving object tracking (max 5km/h)
// <SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY=> Stationary object tracking
// <i> Default: SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC
#define OBJECT_TRACKING_MODE     SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC

// <o INITIATOR_APP_CONFIG_RSSI_TX_POWER> Reference RSSI value of the remote Reflector device at 1.0 m distance in dBm <-100..20>
// <i> Specifes Tx device RSSI at 1m in dBm
// <i> Default: -40
#define INITIATOR_APP_CONFIG_RSSI_REF_TX_POWER               (-40)

// <o CS_INITIATOR_MIN_TX_POWER_DBM> Connection minimum TX Power <-127..20>
// <i> Connection minimum TX Power in dBm
// <i> Default: -3
#define CS_INITIATOR_MIN_TX_POWER_DBM               -3

// <o CS_INITIATOR_MAX_TX_POWER_DBM> Connection maximum TX Power <-127..20>
// <i> Connection minimum TX Power in dBm. Must be greater than the minimum value.
// <i> Default: 20
#define CS_INITIATOR_MAX_TX_POWER_DBM               20

// <<< end of configuration section >>>

#endif // CS_INITIATOR_APP_CONFIG_H
