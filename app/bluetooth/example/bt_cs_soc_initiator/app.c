/***************************************************************************//**
 * @file
 * @brief CS Initiator example application logic
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
// -----------------------------------------------------------------------------
// Includes
#include <stdint.h>
#include <stdbool.h>
#include "sl_bluetooth.h"
#include "sl_component_catalog.h"
#include "em_common.h"
#include "app_assert.h"

// app content
#include "app.h"
#include "initiator_app_config.h"
#include "rtl_log.h"

// initiator content
#include "cs_initiator.h"
#include "cs_initiator_configurator.h"
#include "cs_initiator_config.h"
#include "cs_initiator_display_core.h"
#include "cs_initiator_display.h"
#include "cs_antenna.h"

// other required content
#include "ble_peer_manager_common.h"
#include "ble_peer_manager_connections.h"
#include "ble_peer_manager_central.h"
#include "ble_peer_manager_filter.h"

#ifdef SL_CATALOG_CS_INITIATOR_CLI_PRESENT
#include "cs_initiator_cli.h"
#endif // SL_CATALOG_CS_INITIATOR_CLI_PRESENT

#ifdef SL_CATALOG_SIMPLE_BUTTON_PRESENT
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

// -----------------------------------------------------------------------------
// Macros
#if (SL_SIMPLE_BUTTON_COUNT < 1)
#warning "Selecting CS mode and Object tracking mode with push buttons is not configured!"
#endif
#if (SL_SIMPLE_BUTTON_COUNT == 1)
#warning "Only one push button configured: only CS mode can be selected by push button."
#endif
#endif // SL_CATALOG_SIMPLE_BUTTON_PRESENT

#define MAX_PERCENTAGE                   100u
#define NL                               APP_LOG_NL
#define APP_PREFIX                       "[APP] "
#define INSTANCE_PREFIX                  "[%u] "
#define APP_INSTANCE_PREFIX              APP_PREFIX INSTANCE_PREFIX

// -----------------------------------------------------------------------------
// Static function declarations
static void cs_on_result(const cs_result_t *result, const void *user_data);
static void cs_on_intermediate_result(const cs_intermediate_result_t *intermediate_result,
                                      const void *user_data);
static void cs_on_error(uint8_t conn_handle, cs_error_event_t err_evt, sl_status_t sc);

// -----------------------------------------------------------------------------
// Static variables

#if (SL_SIMPLE_BUTTON_COUNT > 0)
static bool mode_button_pressed = false;
#endif

#if (SL_SIMPLE_BUTTON_COUNT > 1)
static bool algo_mode_button_pressed = false;
#endif

static bool measurement_arrived = false;
static bool measurement_progress_changed = false;
static uint32_t measurement_cnt = 0u;
static cs_initiator_config_t initiator_config;
static rtl_config_t rtl_config;

static cs_result_t measurement;
static cs_intermediate_result_t measurement_progress;

/******************************************************************************
 * Application Init
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  sl_status_t sc = SL_STATUS_OK;
  uint8_t mode = MEASUREMENT_MODE;
  uint8_t algo_mode = OBJECT_TRACKING_MODE;

  app_log_filter_threshold_set(APP_LOG_LEVEL_INFO);
  app_log_filter_threshold_enable(true);
  app_log_info("+-[CS initiator by Silicon Labs]--------------------------+" NL);
  app_log_append_info("+---------------------------------------------------------+" NL);

  app_log_append_info(APP_PREFIX "Default measurement mode: %s" NL,
                      MEASUREMENT_MODE == sl_bt_cs_mode_rtt ? "RTT" : "PBR");

#if (SL_SIMPLE_BUTTON_COUNT > 0)
  app_log_append_info(APP_PREFIX "Press BTN0 while reset to select %s measurement mode!" NL,
                      MEASUREMENT_MODE == sl_bt_cs_mode_rtt ? "PBR" : "RTT");
#endif // (SL_SIMPLE_BUTTON_COUNT > 0)
  app_log_append_info("+---------------------------------------------------------+" NL);
  app_log_append_info(APP_PREFIX "Default object tracking mode: %s" NL,
                      OBJECT_TRACKING_MODE == SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY
                      ? "stationary object tracking"
                      : "moving object tracking    ");
#if (SL_SIMPLE_BUTTON_COUNT > 1)
  app_log_append_info(APP_PREFIX "Press BTN1 while reset to select object tracking mode:" NL);
  app_log_append_info("%s" NL,
                      OBJECT_TRACKING_MODE == SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY
                      ? "moving object tracking    "
                      : "stationary object tracking");
#endif // (SL_SIMPLE_BUTTON_COUNT > 1)

  app_log_append_info("+---------------------------------------------------------+" NL);

  app_log_info(APP_PREFIX "Wire%s antenna offset will be used." NL,
               CS_INITIATOR_ANTENNA_OFFSET ? "d" : "less");

  app_log_info(APP_PREFIX "Minimum CS procedure interval: %u" NL, CS_INITIATOR_MIN_INTERVAL);
  app_log_info(APP_PREFIX "Maximum CS procedure interval: %u" NL, CS_INITIATOR_MAX_INTERVAL);

  // initialize measurement variable
  memset(&measurement, 0u, sizeof(measurement));
  memset(&measurement_progress, 0u, sizeof(measurement_progress));
  measurement.connection = SL_BT_INVALID_CONNECTION_HANDLE;

  rtl_log_init();
  ble_peer_manager_central_init();
  ble_peer_manager_filter_init();
  cs_initiator_init();

  // Default parameters
  cs_initiator_set_default_config(&initiator_config, &rtl_config);

  // Log channel map
  app_log_info("+-[ CS channel map ]------------------------------------+" NL);
  app_log_info("| ");
  for (uint32_t i = 0; i < initiator_config.channel_map_len; i++) {
    app_log_append_info("0x%02x ", initiator_config.channel_map.data[i]);
  }
  app_log_append_info(NL);
  app_log_info("+-------------------------------------------------------+" NL);

#if (SL_SIMPLE_BUTTON_COUNT > 0)
  mode_button_pressed = sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0));

  if (mode_button_pressed == SL_SIMPLE_BUTTON_PRESSED) {
    mode = MEASUREMENT_MODE == sl_bt_cs_mode_pbr
           ? sl_bt_cs_mode_rtt
           : sl_bt_cs_mode_pbr;
  }
  app_log_info(APP_PREFIX "Measurement mode selected: ");
  app_log_append_info("%s" NL, mode == sl_bt_cs_mode_rtt ? "RTT" : "PBR");
#endif // (SL_SIMPLE_BUTTON_COUNT > 0)
  initiator_config.cs_mode = mode;

#if (SL_SIMPLE_BUTTON_COUNT > 1)
  algo_mode_button_pressed = sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(1));

  if (algo_mode_button_pressed == SL_SIMPLE_BUTTON_PRESSED) {
    algo_mode = OBJECT_TRACKING_MODE == SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC
                ? SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY
                : SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC;
  }

  app_log_info(APP_PREFIX "Object tracking mode selected: %s" NL,
               algo_mode == SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY
               ? "stationary object tracking"
               : "moving object tracking");
#endif // (SL_SIMPLE_BUTTON_COUNT > 1)
  rtl_config.algo_mode = algo_mode;

#if defined(SL_CATALOG_CS_INITIATOR_CLI_PRESENT) && defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
  app_log_warning(APP_PREFIX "Measurement mode and Object tracking mode selected by "
                             "push buttons may be overruled by CLI configuration!" NL);
#endif // defined (SL_CATALOG_CS_INITIATOR_CLI_PRESENT) && defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)

  // Show the first LCD screen
  sc = cs_initiator_display_init();

  cs_initiator_display_set_measurement_mode(initiator_config.cs_mode,
                                            rtl_config.algo_mode);

  app_assert_status_f(sc, "cs_initiator_display_init failed");

  app_assert_status_f(sc, "cs_initiator_display_init failed");

  initiator_config.rssi_ref_tx_power = INITIATOR_APP_CONFIG_RSSI_REF_TX_POWER;
  app_log_info(APP_PREFIX "RSSI reference TX power is %d dBm @ 1m" NL,
               (int)initiator_config.rssi_ref_tx_power);

  app_log_info("+-------------------------------------------------------+" NL);
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/******************************************************************************
 * Application Process Action
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  if (measurement_arrived) {
    // write results to the display & to the iostream
    measurement_arrived = false;
    app_log_info("+-------------------------------------------------------+" NL);
    app_log_info(APP_INSTANCE_PREFIX "[Measurement %04lu]" NL,
                 measurement.connection,
                 measurement_cnt);

    app_log_info(APP_INSTANCE_PREFIX "Measurement result: %lu mm" NL,
                 measurement.connection,
                 (uint32_t)(measurement.distance * 1000.f));

    cs_initiator_display_set_distance(measurement.distance);
    cs_initiator_display_set_distance_progress(MAX_PERCENTAGE);

    uint32_t likeliness_whole = (uint32_t)measurement.likeliness;
    uint32_t likeliness_frac =
      (uint32_t)((measurement.likeliness - (float)likeliness_whole) * 100.0f);
    app_log_info(APP_INSTANCE_PREFIX "Measurement likeliness: %lu.%02lu" NL,
                 measurement.connection,
                 likeliness_whole,
                 likeliness_frac);

    cs_initiator_display_set_likeliness(measurement.likeliness);

    app_log_info(APP_INSTANCE_PREFIX "RSSI distance: %lu mm" NL,
                 measurement.connection,
                 (uint32_t)(measurement.rssi_distance * 1000.f));
    cs_initiator_display_set_rssi_distance(measurement.rssi_distance);

    app_log_info(APP_INSTANCE_PREFIX "CS bit error rate: %u" NL,
                 measurement.connection,
                 measurement.cs_bit_error_rate);
    cs_initiator_display_set_bit_error_rate(measurement.cs_bit_error_rate);
    app_log_info("+-------------------------------------------------------+" NL NL);
  } else if (measurement_progress_changed) {
    // write measurement progress to the display without changing the last valid
    // measurement results
    measurement_progress_changed = false;

    app_log_info("+-[I#%u - Measurement %04lu in progress ...]------------+" NL,
                 measurement_progress.connection,
                 measurement_cnt);

    uint32_t percent_whole = (uint32_t)measurement_progress.progress_percentage;
    uint32_t percent_frac =
      (uint32_t)((measurement_progress.progress_percentage - (float)percent_whole) * 100.0f);
    app_log_append_info(APP_INSTANCE_PREFIX " Estimation in progress %lu.%02lu %%" NL,
                        measurement_progress.connection,
                        percent_whole,
                        percent_frac);

    cs_initiator_display_set_distance_progress(measurement_progress.progress_percentage);
    app_log_info("+-------------------------------------------------------+" NL NL);
  }
  rtl_log_step();

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

// -----------------------------------------------------------------------------
// Static function definitions

/******************************************************************************
 * Extract measurement results
 *****************************************************************************/
static void cs_on_result(const cs_result_t *result, const void *user_data)
{
  (void) user_data;
  if (result != NULL) {
    memcpy(&measurement, result, sizeof(measurement));
    measurement_arrived = true;
    measurement_cnt++;
  }
}

/******************************************************************************
 * Extract intermediate results between measurement results
 * Note: only called when stationary object tracking used
 *****************************************************************************/
static void cs_on_intermediate_result(const cs_intermediate_result_t *intermediate_result, const void *user_data)
{
  (void) user_data;
  if (intermediate_result != NULL) {
    memcpy(&measurement_progress, intermediate_result, sizeof(measurement_progress));
    measurement_progress_changed = true;
  }
}

/******************************************************************************
 * CS error handler
 *****************************************************************************/
static void cs_on_error(uint8_t conn_handle, cs_error_event_t err_evt, sl_status_t sc)
{
  switch (err_evt) {
    // Assert
    case CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED:
    case CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA:
      app_assert(false,
                 APP_INSTANCE_PREFIX "Unrecoverable CS procedure error happened!"
                                     "[E: 0x%x sc: 0x%lx]" NL,
                 conn_handle,
                 err_evt,
                 sc);
      break;
    // Discard
    case CS_ERROR_EVENT_RTL_PROCESS_ERROR:
      app_log_error(APP_INSTANCE_PREFIX "RTL processing error happened!"
                                        "[E: 0x%x sc: 0x%lx]" NL,
                    conn_handle,
                    err_evt,
                    sc);
      break;
    // Close connection
    default:
      app_log_error(APP_INSTANCE_PREFIX "Error happened! Closing connection."
                                        "[E: 0x%x sc: 0x%lx]" NL,
                    conn_handle,
                    err_evt,
                    sc);
      // Common errors
      if (err_evt == CS_ERROR_EVENT_TIMER_ELAPSED) {
        app_log_error(APP_INSTANCE_PREFIX "Operation timeout." NL, conn_handle);
      } else if (err_evt == CS_ERROR_EVENT_INITIATOR_FAILED_TO_INCREASE_SECURITY) {
        app_log_error(APP_INSTANCE_PREFIX "Security level increase failed." NL, conn_handle);
      }
      // Close the connection
      (void)ble_peer_manager_central_close_connection(conn_handle);
      break;
  }
}

// -----------------------------------------------------------------------------
// Event / callback definitions

/**************************************************************************//**
 * Bluetooth stack event handler
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  const char* device_name = INITIATOR_DEVICE_NAME;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
    {
      // Set TX power
      int16_t min_tx_power_x10 = CS_INITIATOR_MIN_TX_POWER_DBM * 10;
      int16_t max_tx_power_x10 = CS_INITIATOR_MAX_TX_POWER_DBM * 10;
      sc = sl_bt_system_set_tx_power(min_tx_power_x10,
                                     max_tx_power_x10,
                                     &min_tx_power_x10,
                                     &max_tx_power_x10);
      app_assert_status(sc);
      app_log_info(APP_PREFIX "Minimum system TX power is set to: %d dBm" NL, min_tx_power_x10 / 10);
      app_log_info(APP_PREFIX "Maximum system TX power is set to: %d dBm" NL, max_tx_power_x10 / 10);

      // Print the Bluetooth address
      bd_addr address;
      uint8_t address_type;
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);
      app_log_info(APP_PREFIX "Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);

      // Set antenna offset
      sc = cs_antenna_configure(CS_INITIATOR_ANTENNA_OFFSET);
      app_assert_status(sc);

      // Filter for advertised name (CS_RFLCT)
      sc = ble_peer_manager_set_filter_device_name(device_name,
                                                   strlen(device_name),
                                                   false);
      app_assert_status(sc);

#ifndef SL_CATALOG_CS_INITIATOR_CLI_PRESENT
      sc = ble_peer_manager_central_create_connection();
      app_assert_status(sc);
      // Start scanning for reflector connections
      app_log_info(APP_PREFIX "Scanning started for reflector connections..." NL);
#else
      app_log_info("CS CLI is active." NL);
#endif // SL_CATALOG_CS_INITIATOR_CLI_PRESENT

      break;
    }
    default:
      break;
  }
}

/******************************************************************************
 * BLE peer manager event handler
 *
 * @param[in] evt Event coming from the peer manager.
 *****************************************************************************/
void ble_peer_manager_on_event_initiator(ble_peer_manager_evt_type_t *event)
{
  sl_status_t sc;

  switch (event->evt_id) {
    case BLE_PEER_MANAGER_ON_CONN_OPENED_CENTRAL:
      app_log_info(APP_INSTANCE_PREFIX "Connection opened as central with a CS Reflector" NL, event->connection_id);
#ifdef SL_CATALOG_CS_INITIATOR_CLI_PRESENT
      initiator_config.cs_mode = cs_initiator_cli_get_mode();

      rtl_config.algo_mode = cs_initiator_cli_get_algo_mode();
#endif // SL_CATALOG_CS_INITIATOR_CLI_PRESENT
      sc = cs_initiator_create(event->connection_id,
                               &initiator_config,
                               &rtl_config,
                               cs_on_result,
                               cs_on_intermediate_result,
                               cs_on_error);
      if (sc != SL_STATUS_OK) {
        app_log_error(APP_PREFIX "Failed to create initiator instance, error:0x%lx" NL, sc);
        (void)ble_peer_manager_central_close_connection(event->connection_id);
      } else {
        app_log_info(APP_INSTANCE_PREFIX "New initiator instance created" NL, event->connection_id);
      }
      measurement_cnt = 0u;
      break;

    case BLE_PEER_MANAGER_ON_CONN_CLOSED:
      app_log_info(APP_PREFIX "Connection closed." NL);
      sc = cs_initiator_delete(event->connection_id);
      // Start scanning for reflector connections
      sc = ble_peer_manager_central_create_connection();
      app_assert_status(sc);
      app_log_info(APP_PREFIX "Scanning started for reflector connections..." NL);
      break;

    case BLE_PEER_MANAGER_ERROR:
      app_log_info(APP_PREFIX "Error on connection %u!" NL,
                   event->connection_id);
      break;

    default:
      app_log_info(APP_PREFIX "Unhandled event on connection %u! [evt: 0x%x]" NL,
                   event->connection_id,
                   event->evt_id);
      break;
  }
}
