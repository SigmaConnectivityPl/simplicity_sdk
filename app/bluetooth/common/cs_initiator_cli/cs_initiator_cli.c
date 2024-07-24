/***************************************************************************//**
 * @file
 * @brief CS Initiator command line interface
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

#include "sl_bt_api.h"
#include "sl_status.h"
#include "sl_cli.h"
#include "app_log.h"
#include "sl_rtl_clib_api.h"
#include "ble_peer_manager_filter.h"
#include "ble_peer_manager_central.h"

// -----------------------------------------------------------------------------
// Static variables
static bool start = false;
static uint8_t mode = sl_bt_cs_mode_pbr;
static uint8_t algo_mode = SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC;

/*******************************************************************************
 * CLI Getter for CS mode
 ******************************************************************************/
uint8_t cs_initiator_cli_get_mode(void)
{
  return mode;
}

/*******************************************************************************
 * CLI Getter for CS algorithm mode
 ******************************************************************************/
uint8_t cs_initiator_cli_get_algo_mode(void)
{
  return algo_mode;
}

/*******************************************************************************
 * CLI Callback for "mode" command
 * @param[in] arguments pointer to CLI arguments
 ******************************************************************************/
void cs_initiator_cli_initiator_mode(sl_cli_command_arg_t *arguments)
{
  size_t arg_data;
  sl_cli_get_argument_hex(arguments, 0, &arg_data);
  if (arg_data == sl_bt_cs_mode_pbr || arg_data == sl_bt_cs_mode_rtt) {
    mode = arg_data;
    app_log("OK. Initiator mode set to: %d" APP_LOG_NL, arg_data);
  } else {
    app_log("ERROR. Mode should be %d or %d" APP_LOG_NL,
            sl_bt_cs_mode_pbr, sl_bt_cs_mode_rtt);
  }
}

/*******************************************************************************
 * CLI Callback for "algo_mode" command
 * @param[in] arguments pointer to CLI arguments
 ******************************************************************************/
void cs_initiator_cli_initiator_algo_mode(sl_cli_command_arg_t *arguments)
{
  size_t arg_data;
  sl_cli_get_argument_hex(arguments, 0, &arg_data);
  if (arg_data == SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC) {
    app_log("OK. Initiator object tracking mode set: real time basic "
            "(moving object tracking)." APP_LOG_NL);
    algo_mode = arg_data;
  } else if (arg_data == SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY) {
    app_log("OK. Initiator object tracking mode set: static high accuracy "
            "(stationary object tracking)." APP_LOG_NL);
    algo_mode = arg_data;
  } else {
    app_log("ERROR. Object tracking mode should be %d or %d" APP_LOG_NL,
            SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC,
            SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY);
  }
}

/*******************************************************************************
 * CLI Callback for "filter_address" command
 * @param[in] arguments pointer to CLI arguments
 ******************************************************************************/
void cs_initiator_cli_initiator_filter_address(sl_cli_command_arg_t *arguments)
{
  char *address_str;
  bd_addr ble_address;
  sl_status_t sc;
  address_str = sl_cli_get_argument_string(arguments, 0);
  sc = ble_peer_manager_str_to_address(address_str, &ble_address);
  if (sc != SL_STATUS_OK) {
    app_log("ERROR. Failed to parse BLE address." APP_LOG_NL);
    return;
  }
  ble_peer_manager_set_filter_bt_address(true);
  ble_peer_manager_add_allowed_bt_address(&ble_address);
  app_log("OK." APP_LOG_NL);
}

/*******************************************************************************
 * CLI Callback for "log_level" command
 * @param[in] arguments pointer to CLI arguments
 ******************************************************************************/
void cs_initiator_cli_initiator_log_level(sl_cli_command_arg_t *arguments)
{
  size_t arg_data;
  sl_status_t sc;
  sl_cli_get_argument_hex(arguments, 0, &arg_data);
  app_log_filter_threshold_enable(true);
  sc = app_log_filter_threshold_set((uint8_t)arg_data);
  if (sc != SL_STATUS_OK) {
    app_log("ERROR. Wrong filter." APP_LOG_NL);
  } else {
    app_log("OK." APP_LOG_NL);
  }
}

/*******************************************************************************
 * CLI Callback for "start" command
 * @param[in] arguments pointer to CLI arguments
 ******************************************************************************/
void cs_initiator_cli_initiator_start(sl_cli_command_arg_t *arguments)
{
  (void)arguments;
  sl_status_t sc;
  if (start == false) {
    sc = ble_peer_manager_central_create_connection();
    if (sc != SL_STATUS_OK) {
      app_log("ERROR. Scanning not started." APP_LOG_NL);
    } else {
      app_log("OK." APP_LOG_NL);
      start = true;
    }
  } else {
    app_log("ERROR. Initiator already started." APP_LOG_NL);
  }
}
