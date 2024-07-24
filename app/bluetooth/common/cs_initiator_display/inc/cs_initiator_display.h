/***************************************************************************//**
 * @file
 * @brief CS Initiator display API
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
#ifndef CS_INITIATOR_DISPLAY_H
#define CS_INITIATOR_DISPLAY_H

/***********************************************************************************************//**
 * @addtogroup cs_initiator_display
 * @{
 **************************************************************************************************/

// -----------------------------------------------------------------------------
// Includes

#include "sl_bt_api.h"
#include "sl_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

// -----------------------------------------------------------------------------
// Macros

#define CS_INITIATOR_DISPLAY_TITLE_VENDOR_TEXT          "Silicon Labs"
#define CS_INITIATOR_DISPLAY_TITLE_DEVICE_TEXT          "CS Initiator"

#define CS_INITIATOR_DISPLAY_MODE_TEXT                  "Mode:"
#define CS_INITIATOR_DISPLAY_DISTANCE_TEXT              "Distance:"
#define CS_INITIATOR_DISPLAY_RSSI_DISTANCE_TEXT         "Distance [RSSI]:"
#define CS_INITIATOR_DISPLAY_LIKELINESS_TEXT            "Likeliness:"
#define CS_INITIATOR_DISPLAY_BER_TEXT                   "Bit error:"

#define CS_INITIATOR_DISPLAY_STATE_ESTIMATE_TEXT        "ST: Estimate"
#define CS_INITIATOR_DISPLAY_STATE_CONNECTED_TEXT       "ST: Connected"
#define CS_INITIATOR_DISPLAY_STATE_DISCONNECTED_TEXT    "ST: Disconnected"
#define CS_INITIATOR_DISPLAY_STATE_SCANNING_TEXT        "ST: Scanning..."
#define CS_INITIATOR_DISPLAY_STATE_INITIALIZED_TEXT     "ST: Initialized"

#define CS_INITIATOR_DISPLAY_MODE_RTT_TEXT              "RTT"
#define CS_INITIATOR_DISPLAY_MODE_PBR_TEXT              "PBR"
#define CS_INITIATOR_DISPLAY_AMODE_STATIONARY_OBJ_TEXT  " [stationary]"
#define CS_INITIATOR_DISPLAY_AMODE_MOVING_OBJ_TEXT      " [moving    ]"

// -----------------------------------------------------------------------------
// Enums, structs, typedefs

typedef struct {
  sl_bt_cs_mode_t mode;
  uint8_t algo_mode;
  float distance;
  float rssi_distance;
  float progress_percentage;
  float likeliness;
  float bit_error_rate;
} cs_initiator_display_content_t;

// -----------------------------------------------------------------------------
// Function declarations

// -----------------------------------------------------------------------------
// Setters

/**************************************************************************//**
 * Set distance value to display
 * @param[in] distance Distance value
 *****************************************************************************/
void cs_initiator_display_set_distance(float distance);

/**************************************************************************//**
 * Set distance progress percentage to display
 * @param[in] progress_percentage Distance progress percentage
 *****************************************************************************/
void cs_initiator_display_set_distance_progress(float progress_percentage);

/**************************************************************************//**
 * Set RSSI based distance value to display
 * @param[in] rssi RSSI distance value
 *****************************************************************************/
void cs_initiator_display_set_rssi_distance(float distance);

/**************************************************************************//**
 * Set the likeliness parameter to display
 * @param[in] likeliness likeliness value
 *****************************************************************************/
void cs_initiator_display_set_likeliness(float likeliness);

/**************************************************************************//**
 * Set the Bit Error Rate (BER) value to display
 * @param[in] ber BER value
 *****************************************************************************/
void cs_initiator_display_set_bit_error_rate(float ber);

/**************************************************************************//**
 * Set the measurement mode and object tracking mode to display
 *
 * @param[in] mode CS measurement mode.
 * @param[in] algo_mode Object tracking mode.
 *****************************************************************************/
void cs_initiator_display_set_measurement_mode(sl_bt_cs_mode_t mode,
                                               uint8_t algo_mode);

/**************************************************************************//**
 * Print floating point value on the LCD with an optional unit string.
 *
 * @param[in] value the calculated value
 * @param[in] row number of the row to print on, see cs_initiator_display_row_t
 * @param[in] unit unit string, in case of NULL no unit displayed
 *****************************************************************************/
void cs_initiator_display_print_float_value(float value,
                                            uint8_t row,
                                            char *unit);

// -----------------------------------------------------------------------------
// Event / callback declarations

/**************************************************************************//**
 * Bluetooth stack event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void cs_initiator_display_on_event(sl_bt_msg_t *evt);

#ifdef __cplusplus
}
#endif

/** @} (end addtogroup cs_initiator_display) */
#endif // CS_INITIATOR_DISPLAY_H
