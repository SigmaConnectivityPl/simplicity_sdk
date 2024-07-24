/***************************************************************************//**
 * @file
 * @brief CS NCP host
 *
 * Reference implementation of a CS host with initiator and reflector support.
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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "ncp_host.h"
#include "app_log.h"
#include "app_log_cli.h"
#include "app_assert.h"
#include "sl_bt_api.h"
#include "host_comm.h"
#include "gatt_db.h"
#include "rtl_log.h"
#include "ble_peer_manager_common.h"
#include "ble_peer_manager_connections.h"
#include "ble_peer_manager_central.h"
#include "ble_peer_manager_peripheral.h"
#include "ble_peer_manager_filter.h"
#include "cs_reflector.h"
#include "cs_reflector_config.h"
#include "cs_acp.h"
#include "cs_initiator_configurator.h"
#include "cs_antenna.h"
#include "cs_log.h"

// MAX_CONNECTIONS should be less or equal to the SL_BT_CONFIG_MAX_CONNECTIONS in the CS NCP example.
// Sum of reflector and initiator instances should not exceed MAX_CONNECTIONS.
#define MAX_CONNECTIONS                             4u
// MAX_INITIATOR_INSTANCES should be less or equal to the CS_INITIATOR_MAX_CONNECTIONS in the CS NCP example.
#define MAX_INITIATOR_INSTANCES                     MAX_CONNECTIONS
// MAX_REFLECTOR_INSTANCES should be less or equal to the CS_REFLECTOR_MAX_CONNECTIONS in the CS NCP example.
#define MAX_REFLECTOR_INSTANCES                     MAX_CONNECTIONS

// Prefixes
#define APP_PREFIX                  "[APP] "
#define INSTANCE_PREFIX             "[%u] "
#define APP_INSTANCE_PREFIX         APP_PREFIX INSTANCE_PREFIX

// Reference RSSI value of the remote Reflector device at 1.0 m distance in dBm
#define INITIATOR_APP_CONFIG_RSSI_REF_TX_POWER      -40

// Connection TX power settings
#define CS_INITIATOR_MIN_TX_POWER_DBM               -3
#define CS_INITIATOR_MAX_TX_POWER_DBM               20

#define CREATE_REFLECTOR_MSG_LEN (sizeof(cs_acp_cmd_id_t) + sizeof(cs_acp_create_reflector_cmd_data_t))
#define REFLECTOR_ACTION_MSG_LEN (sizeof(cs_acp_cmd_id_t) + sizeof(cs_acp_reflector_action_cmd_data_t))

#define CREATE_INITATOR_MSG_LEN (sizeof(cs_acp_cmd_id_t) + sizeof(cs_acp_create_initiator_cmd_data_t))
#define INITATOR_ACTION_MSG_LEN (sizeof(cs_acp_cmd_id_t) + sizeof(cs_acp_initiator_action_cmd_data_t))

static uuid_128 ras_service_uuid = {
  .data = { 0xf3, 0x20, 0x18, 0xc7, 0x32, 0x2d, 0xc7, 0xab, 0xcf, 0x46, 0xf7, 0xff, 0x70, 0x9e, 0xb9, 0xbb }
};

// Optstring argument for getopt
#define OPTSTRING NCP_HOST_OPTSTRING APP_LOG_OPTSTRING "hm:R:I:F:rwo:"

// Usage info
#define USAGE APP_LOG_NL "%s " NCP_HOST_USAGE APP_LOG_USAGE                                                      \
  "\n[-m <cs_mode>] [-R <max_reflector_instances>] [-I <max_initiator_instances>] [-F <reflector_ble_address>] " \
  "[-w] [-o] [-h]" APP_LOG_NL

// Detailed argument list
#define CS_HOST_INITIATOR_OPTIONS                                           \
  "    -m  Number corresponding to the selected CS mode, default: 2, PBR\n" \
  "        <cs_mode>\n"                                                     \
  "        1 : RTT\n"                                                       \
  "        2 : PBR\n"                                                       \
  "    -R  Maximum number of reflector instances, default: 1\n"             \
  "        <max_reflector_instances>\n"                                     \
  "    -I  Maximum number of initiator instances, default: 1\n"             \
  "        <max_initiator_instances>\n"                                     \
  "    -F  Enable reflector BLE address filtering in format:\n"             \
  "        AA:BB:CC:DD:EE:FF or aabbccddeeff\n"                             \
  "        <reflector_ble_address>\n"                                       \
  "    -w  Use wired antenna offset\n"                                      \
  "    -o  Object tracking mode, default: 0\n"                              \
  "        0 : moving object tracking (up to 5 km/h) (REAL_TIME_BASIC)\n"   \
  "        1 : stationary object tracking (STATIC_HIGH_ACCURACY)\n"

// Options info
#define OPTIONS             \
  "\nOPTIONS\n"             \
  NCP_HOST_OPTIONS          \
  APP_LOG_OPTIONS           \
  CS_HOST_INITIATOR_OPTIONS \
  "    -h  Print this help message.\n"

#define BT_ADDR_LEN 6u

typedef struct {
  sl_bt_cs_mode_t cs_mode;
  uint8_t object_tracking_mode;
  bool use_antenna_wired_offset;
  bd_addr accepted_bt_address_list[MAX_REFLECTOR_INSTANCES];
  uint32_t accepted_bt_address_count;
  uint32_t max_reflector_instances;
  uint32_t max_initiator_instances;
} cs_host_config_t;

typedef struct {
  uint32_t num_reflector_connections;
  uint32_t num_initiator_connections;
  uint8_t reflector_conn_handles[MAX_REFLECTOR_INSTANCES];
  uint8_t initiator_conn_handles[MAX_INITIATOR_INSTANCES];
} cs_host_state_t;

cs_host_config_t cs_host_config = {
  .cs_mode = sl_bt_cs_mode_pbr,
  .object_tracking_mode = SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC,
  .use_antenna_wired_offset = false,
  .accepted_bt_address_count = 0u,
  .max_reflector_instances = 0u,
  .max_initiator_instances = 0u
};

cs_host_state_t cs_host_state = {
  .num_reflector_connections = 0u,
  .num_initiator_connections = 0u,
  .reflector_conn_handles = { SL_BT_INVALID_CONNECTION_HANDLE },
  .initiator_conn_handles = { SL_BT_INVALID_CONNECTION_HANDLE }
};

void cs_on_result(cs_result_t *result, void *user_data);
void cs_on_intermediate_result(const cs_intermediate_result_t *intermediate_result,
                               const void *user_data);
void cs_on_error(uint8_t conn_handle, cs_error_event_t err_evt, sl_status_t sc);
void handle_connection_opened_with_initiator(uint8_t conn_handle);
void handle_connection_opened_with_reflector(uint8_t conn_handle);
void handle_connection_closed(uint8_t conn_handle);
void stop_procedure(void);

/******************************************************************************
 * Application initialization
 *****************************************************************************/
void app_init(int argc, char *argv[])
{
  sl_status_t sc;
  int cli_opt;
  int cs_mode = sl_bt_cs_mode_pbr;
  bool max_initiator_instances_set = false;
  bool max_reflector_instances_set = false;

  // Initialize the list of accepted BT addresses
  for (uint32_t i = 0u; i < MAX_REFLECTOR_INSTANCES; i++) {
    memset(cs_host_config.accepted_bt_address_list[i].addr, 0xFF, BT_ADDR_LEN);
  }

  app_log_info("+-[CS Host by Silicon Labs]------------------------+" APP_LOG_NL);
  app_log_info("+--------------------------------------------------+" APP_LOG_NL APP_LOG_NL);

  // Process command line options
  while ((cli_opt = getopt(argc, argv, OPTSTRING)) != -1) {
    switch (cli_opt) {
      // Print help
      case 'h':
        app_log(USAGE, argv[0]);
        app_log(OPTIONS);
        exit(EXIT_SUCCESS);

      case 'm':
        // Mode.
        // 1 - RTT
        // 2 - Phase based
        cs_mode = atoi(optarg);
        break;

      case 'R':
        cs_host_config.max_reflector_instances = atoi(optarg);
        max_reflector_instances_set = true;
        if (cs_host_config.max_reflector_instances > MAX_REFLECTOR_INSTANCES) {
          app_log_error(APP_PREFIX "Invalid number of 'maximum reflectors' argument (%d) provided, "
                                   "must be in the range of 0 to %u" APP_LOG_NL,
                        cs_host_config.max_reflector_instances,
                        MAX_REFLECTOR_INSTANCES);
          exit(EXIT_FAILURE);
        }
        break;

      case 'I':
        cs_host_config.max_initiator_instances = atoi(optarg);
        max_initiator_instances_set = true;
        if (cs_host_config.max_initiator_instances > MAX_INITIATOR_INSTANCES) {
          app_log_error(APP_PREFIX "Invalid number of 'maximum initiators' argument (%d) provided, "
                                   "must be in the range of 0 to %u" APP_LOG_NL,
                        cs_host_config.max_initiator_instances,
                        MAX_INITIATOR_INSTANCES);
          exit(EXIT_FAILURE);
        }
        break;

      case 'F':
      {
        // Check if we have room for more accepted addresses
        if (cs_host_config.accepted_bt_address_count >= MAX_REFLECTOR_INSTANCES) {
          app_log_error(APP_PREFIX "Maximum number of accepted BLE addresses (%u) reached, "
                                   "ignoring additional addresses" APP_LOG_NL, MAX_REFLECTOR_INSTANCES);
          break;
        }
        // Add the accepted BLE address to the list
        bd_addr *bt_addr_to_add = &cs_host_config.accepted_bt_address_list[cs_host_config.accepted_bt_address_count];
        sc = ble_peer_manager_str_to_address(optarg, bt_addr_to_add);
        if (sc != SL_STATUS_OK) {
          app_log_error(APP_PREFIX "Invalid BLE address filter provided" APP_LOG_NL);
          exit(EXIT_FAILURE);
        }
        app_log_info(APP_PREFIX "BLE address accept filter added for: '%02x:%02x:%02x:%02x:%02x:%02x'" APP_LOG_NL,
                     bt_addr_to_add->addr[5],
                     bt_addr_to_add->addr[4],
                     bt_addr_to_add->addr[3],
                     bt_addr_to_add->addr[2],
                     bt_addr_to_add->addr[1],
                     bt_addr_to_add->addr[0]);
        cs_host_config.accepted_bt_address_count++;
      }
      break;

      case 'w':
        cs_host_config.use_antenna_wired_offset = true;
        break;
      case 'o':
      {
        int object_tracking_mode = atoi(optarg);

        if (object_tracking_mode != SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC
            && object_tracking_mode != SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY) {
          app_log_error(APP_PREFIX "Invalid object tracking mode (%d) provided!" APP_LOG_NL, object_tracking_mode);
          exit(EXIT_FAILURE);
        } else {
          cs_host_config.object_tracking_mode = (uint8_t)object_tracking_mode;
        }
      }
      break;
      default:
        sc = ncp_host_set_option((char)cli_opt, optarg);
        if (sc == SL_STATUS_NOT_FOUND) {
          sc = app_log_set_option((char)cli_opt, optarg);
        }
        if (sc != SL_STATUS_OK) {
          app_log(USAGE, argv[0]);
          exit(EXIT_FAILURE);
        }
        break;
    }
  }

  // Check for unknown parameters and write them to to console if there is any
  if (optind < argc) {
    app_log_critical("Unknown %d parameter", (argc - optind));
    if (argc - optind > 1) {
      app_log_append_critical("s \'");
      for (int i = optind; i < argc; i++) {
        app_log_append_critical("%s", argv[i]);
        if (i < argc - 1) {
          app_log_append_critical(", ");
        }
      }
      app_log_append_critical("\'");
    } else {
      app_log_append_critical(" \'%s\'", argv[optind]);
    }
    app_log_append_critical("! Usage:\n");

    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }

  // Sanity check of argument combinations
  if ((cs_host_config.max_initiator_instances + cs_host_config.max_reflector_instances) > MAX_CONNECTIONS) {
    app_log_info(APP_PREFIX "Sum of <max_initiator_instances> and <max_reflector_instances> "
                            "exceeds maximum connection count (%u)" APP_LOG_NL,
                 MAX_CONNECTIONS);
    exit(EXIT_FAILURE);
  }

  if (cs_host_config.max_initiator_instances == 0 && cs_host_config.max_reflector_instances == 0) {
    if (!max_initiator_instances_set && !max_reflector_instances_set) {
      cs_host_config.max_initiator_instances = 1;
      cs_host_config.max_reflector_instances = 1;
      app_log_info(APP_PREFIX "Not specified <max_initiator_instances> and "
                              "<max_reflector_instances>. Using 1-1 of each" APP_LOG_NL);
    } else {
      app_log_error(APP_PREFIX "<max_initiator_instances> and <max_reflector_instances>"
                               " cannot be both zero" APP_LOG_NL);
      exit(EXIT_FAILURE);
    }
  }

  if (cs_host_config.max_initiator_instances > 0) {
    if (cs_mode == (int)sl_bt_cs_mode_rtt) {
      cs_host_config.cs_mode = sl_bt_cs_mode_rtt;
      app_log_info(APP_PREFIX "CS mode: 1 - RTT" APP_LOG_NL);
    } else if (cs_mode == (int)sl_bt_cs_mode_pbr) {
      cs_host_config.cs_mode = sl_bt_cs_mode_pbr;
      app_log_info(APP_PREFIX "CS mode: 2 - PBR" APP_LOG_NL);
    } else {
      app_log_error(APP_PREFIX "Invalid CS mode argument (%d) provided" APP_LOG_NL, cs_mode);
      exit(EXIT_FAILURE);
    }
  }

  if (cs_host_config.accepted_bt_address_count > 0) {
    app_log_info(APP_PREFIX "BLE address filtering enabled with %u specified BT addresses" APP_LOG_NL,
                 cs_host_config.accepted_bt_address_count);
  } else {
    app_log_info(APP_PREFIX "BLE address filtering disabled" APP_LOG_NL);
  }

  app_log_info(APP_PREFIX "Maximum number of reflector instances: %u" APP_LOG_NL,
               cs_host_config.max_reflector_instances);
  app_log_info(APP_PREFIX "Maximum number of initiator instances: %u" APP_LOG_NL,
               cs_host_config.max_initiator_instances);

  app_log_info(APP_PREFIX "Tracking mode: ");
  if (cs_host_config.object_tracking_mode == SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC) {
    app_log_append_info("moving objects" APP_LOG_NL);
  } else {
    app_log_append_info("stationary objects" APP_LOG_NL);
  }

  app_log_info(APP_PREFIX "RSSI reference TX power is %d dBm @ 1m" APP_LOG_NL,
               (int)INITIATOR_APP_CONFIG_RSSI_REF_TX_POWER);

  // Initialize the list of reflector connection handles
  for (uint32_t i = 0u; i < cs_host_config.max_initiator_instances; i++) {
    cs_host_state.reflector_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
  }
  // Initialize the list of initiator connection handles
  for (uint32_t i = 0u; i < cs_host_config.max_reflector_instances; i++) {
    cs_host_state.initiator_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
  }

  // Initialize the NCP connection
  sc = ncp_host_init();
  if (sc == SL_STATUS_INVALID_PARAMETER) {
    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }
  app_assert_status(sc);
  app_log_info(APP_PREFIX "NCP host initialized" APP_LOG_NL);
  app_log_info(APP_PREFIX "Press Crtl+C to quit" APP_LOG_NL);
  app_log_info("+-------------------------------------------------------+" APP_LOG_NL APP_LOG_NL);

  cs_log_create(argc, argv);
  rtl_log_init();
  ble_peer_manager_central_init();
  ble_peer_manager_peripheral_init();
  ble_peer_manager_filter_init();
}

void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  const char *device_name = INITIATOR_DEVICE_NAME;
  switch (SL_BT_MSG_ID(evt->header)) {
    // --------------------------------
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
      app_log_info(APP_PREFIX "Minimum system TX power is set to: %d dBm" APP_LOG_NL, min_tx_power_x10 / 10);
      app_log_info(APP_PREFIX "Maximum system TX power is set to: %d dBm" APP_LOG_NL, max_tx_power_x10 / 10);

      // Reset to initial state
      ble_peer_manager_central_init();
      ble_peer_manager_peripheral_init();
      ble_peer_manager_filter_init();
      cs_host_state.num_reflector_connections = 0u;
      cs_host_state.num_initiator_connections = 0u;

      // Initialize the list of reflector connection handles
      for (uint32_t i = 0u; i < cs_host_config.max_initiator_instances; i++) {
        cs_host_state.reflector_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
      }
      // Initialize the list of initiator connection handles
      for (uint32_t i = 0u; i < cs_host_config.max_reflector_instances; i++) {
        cs_host_state.initiator_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
      }

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

      // Filter for advertised name (CS_RFLCT)
      sc = ble_peer_manager_set_filter_device_name(device_name, strlen(device_name), false);
      app_assert_status(sc);

      // Filter for advertised service UUID (RAS)
      sc = ble_peer_manager_set_filter_service_uuid128(&ras_service_uuid);
      app_assert_status(sc);

      // Filter for BLE address if enabled
      if (cs_host_config.accepted_bt_address_count > 0) {
        // Enable Peer Manager BT address filtering
        ble_peer_manager_set_filter_bt_address(true);
        // Add all user specified BT addresses to the allowed list
        for (uint32_t i = 0u; i < cs_host_config.accepted_bt_address_count; i++) {
          sc = ble_peer_manager_add_allowed_bt_address(&cs_host_config.accepted_bt_address_list[i]);
          app_assert_status(sc);
        }
      }

      // Start scanning for reflector connections
      if (cs_host_config.max_initiator_instances > 0 ) {
        sc = ble_peer_manager_central_create_connection();
        app_assert_status(sc);
        app_log_info(APP_PREFIX "Scanning started for reflector connections..." APP_LOG_NL);
      }

      // Start advertising for initiator connections
      if (cs_host_config.max_reflector_instances > 0 ) {
        sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
        app_assert_status(sc);
        app_log_info(APP_PREFIX "Advertising started for initiator connections..." APP_LOG_NL);
      }

      // Set antenna configuration
      sc = cs_antenna_configure(cs_host_config.use_antenna_wired_offset);
      app_assert_status(sc);

      break;
    }

    // --------------------------------
    case sl_bt_evt_user_cs_service_message_to_host_id:
    {
      cs_acp_event_t *cs_evt = (cs_acp_event_t *)evt->data.evt_user_cs_service_message_to_host.message.data;
      switch (cs_evt->acp_evt_id) {
        case CS_ACP_EVT_LOG_DATA_ID:
          switch (cs_evt->data.log.src) {
            case LOG_SRC_RTL:
              rtl_log_on_event(&cs_evt->data.log);
              break;
            case LOG_SRC_INITIATOR:
              app_log(APP_PREFIX "[INITIATOR] %s" APP_LOG_NL, cs_evt->data.log.log_data.data);
              break;
            case LOG_SRC_REFLECTOR:
              app_log(APP_PREFIX "[REFLECTOR] %s" APP_LOG_NL, cs_evt->data.log.log_data.data);
              break;
            default:
              app_log_warning(APP_PREFIX "Unknown log source: %u" APP_LOG_NL, cs_evt->data.log.src);
              break;
          }
          break;
        case CS_ACP_EVT_RESULT_ID:
        {
          cs_result_t cs_result = {
            .connection = cs_evt->connection_id,
            .cs_bit_error_rate = cs_evt->data.result.bit_error_rate,
            .distance = cs_evt->data.result.estimated_distance_mm,
            .likeliness = cs_evt->data.result.likeliness,
            .rssi_distance = cs_evt->data.result.rssi_distance_mm
          };
          cs_on_result(&cs_result, NULL);
          break;
        }

        case CS_ACP_EVT_INTERMEDIATE_RESULT_ID:
        {
          cs_intermediate_result_t cs_intermediate_result = {
            .connection = cs_evt->connection_id,
            .progress_percentage = cs_evt->data.intermediate_result.progress_percentage
          };
          cs_on_intermediate_result(&cs_intermediate_result, NULL);
          break;
        }

        case CS_ACP_EVT_STATUS_ID:
          cs_on_error(cs_evt->connection_id, cs_evt->data.stat.error, cs_evt->data.stat.sc);
          break;
        default:
          app_log(APP_PREFIX "Unknown ACP event" APP_LOG_NL);
          break;
      }
      break;
    }
  }
}

/******************************************************************************
 * Application process action
 *****************************************************************************/
void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/******************************************************************************
 * Application deinit
 *****************************************************************************/
void app_deinit(void)
{
  rtl_log_deinit();
  stop_procedure();
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application deinit code here!                       //
  // This is called once during termination.                                 //
  /////////////////////////////////////////////////////////////////////////////
}

void handle_connection_opened_with_reflector(uint8_t conn_handle)
{
  sl_status_t sc;

  // Check if we can accept one more reflector connection
  if (cs_host_state.num_reflector_connections >= cs_host_config.max_initiator_instances) {
    app_log_error(APP_PREFIX "Maximum number of initiator instances (%u) reached, "
                             "dropping connection..." APP_LOG_NL,
                  cs_host_config.max_initiator_instances);
    sc = sl_bt_connection_close(conn_handle);
    app_assert_status(sc);
    return;
  }

  // Store the new reflector connection handle
  bool could_store = false;
  for (uint32_t i = 0u; i < cs_host_config.max_initiator_instances; i++) {
    if (cs_host_state.reflector_conn_handles[i] == SL_BT_INVALID_CONNECTION_HANDLE) {
      cs_host_state.reflector_conn_handles[i] = conn_handle;
      cs_host_state.num_reflector_connections++;
      could_store = true;
      break;
    }
  }
  if (!could_store) {
    app_log_error(APP_PREFIX "Failed to store reflector connection handle" APP_LOG_NL);
    return;
  }

  cs_acp_create_initiator_cmd_data_t initiator_cmd_data;

  initiator_cmd_data.connection_id = conn_handle;
  cs_initiator_set_default_config(&initiator_cmd_data.initiator_config, &initiator_cmd_data.rtl_config);
  initiator_cmd_data.rtl_config.algo_mode = cs_host_config.object_tracking_mode;
  initiator_cmd_data.initiator_config.cs_mode = cs_host_config.cs_mode;
  initiator_cmd_data.initiator_config.rssi_ref_tx_power = INITIATOR_APP_CONFIG_RSSI_REF_TX_POWER;

  cs_acp_cmd_t acp_cmd;
  // Construct an ACP command
  acp_cmd.cmd_id = CS_ACP_CMD_CREATE_INITIATOR;
  acp_cmd.data.initiator_cmd_data = initiator_cmd_data;

  // Start the Initiator
  sc = sl_bt_user_cs_service_message_to_target(CREATE_INITATOR_MSG_LEN,
                                               (uint8_t *)&acp_cmd,
                                               0,
                                               NULL,
                                               NULL);

  if (sc != SL_STATUS_OK) {
    app_log_status_f(sc, "Failed to create initiator instance" APP_LOG_NL);
    return;
  }

  app_log_info(APP_INSTANCE_PREFIX "New initiator instance created" APP_LOG_NL, conn_handle);

  app_log_info(APP_PREFIX "Initiator instances in use: %u/%u" APP_LOG_NL,
               cs_host_state.num_reflector_connections,
               cs_host_config.max_initiator_instances);

  // Scan for new reflector connections if we have room for more
  if (cs_host_state.num_reflector_connections < cs_host_config.max_initiator_instances) {
    sl_status_t sc = ble_peer_manager_central_create_connection();
    app_assert_status(sc);
    app_log_info(APP_PREFIX "Scanning restarted for new reflector connections..." APP_LOG_NL);
  }
}

void handle_connection_opened_with_initiator(uint8_t conn_handle)
{
  // Check if we can accept one more initiator connection
  if (cs_host_state.num_initiator_connections >= cs_host_config.max_reflector_instances) {
    app_log_error(APP_PREFIX "Maximum number of reflector instances (%u) reached, "
                             "dropping connection..." APP_LOG_NL,
                  cs_host_config.max_reflector_instances);
    sl_status_t sc = sl_bt_connection_close(conn_handle);
    app_assert_status(sc);
    return;
  }

  // Store the new initiator connection handle
  for (uint32_t i = 0u; i < cs_host_config.max_reflector_instances; i++) {
    if (cs_host_state.initiator_conn_handles[i] == SL_BT_INVALID_CONNECTION_HANDLE) {
      cs_host_state.initiator_conn_handles[i] = conn_handle;
      cs_host_state.num_initiator_connections++;
      break;
    }
  }
  // Call the ACP API to create a new reflector instance for the connection handle
  sl_status_t sc;
  cs_acp_cmd_t msg;

  msg.cmd_id = CS_ACP_CMD_CREATE_REFLECTOR;
  msg.data.reflector_cmd_data.connection_id = conn_handle;
  msg.data.reflector_cmd_data.reflector_config.max_tx_power_dbm = CS_REFLECTOR_MAX_TX_POWER_DBM;
  sc = sl_bt_user_cs_service_message_to_target(CREATE_REFLECTOR_MSG_LEN,
                                               (uint8_t *)&msg,
                                               0,
                                               NULL,
                                               NULL);
  app_assert_status(sc);
  app_log_info(APP_INSTANCE_PREFIX "New reflector instance created" APP_LOG_NL, conn_handle);

  app_log_info(APP_PREFIX "Reflector instances in use: %u/%u" APP_LOG_NL,
               cs_host_state.num_initiator_connections,
               cs_host_config.max_reflector_instances);

  // Advertise for new initiator connections if we have room for more
  if (cs_host_state.num_initiator_connections < cs_host_config.max_reflector_instances) {
    sl_status_t sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
    app_assert_status(sc);
    app_log_info(APP_PREFIX "Advertising restarted for new initiator connections..." APP_LOG_NL);
  }
}

void handle_connection_closed(uint8_t conn_handle)
{
  // Check if the connection handle is a reflector connection - if yes remove the corresponding initiator instance
  for (uint32_t i = 0u; i < cs_host_config.max_initiator_instances; i++) {
    if (cs_host_state.reflector_conn_handles[i] == conn_handle) {
      cs_host_state.reflector_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
      app_log_info(APP_INSTANCE_PREFIX "Reflector connection closed" APP_LOG_NL, conn_handle);

      // Call the ACP API to remove the initiator instance for the connection handle
      cs_acp_cmd_t acp_cmd;
      sl_status_t sc;

      acp_cmd.cmd_id = CS_ACP_CMD_INITIATOR_ACTION;
      acp_cmd.data.initiator_action_data.connection_id = conn_handle;
      acp_cmd.data.initiator_action_data.initiator_action = CS_ACP_ACTION_DELETE_INITIATOR;
      sc = sl_bt_user_cs_service_message_to_target(INITATOR_ACTION_MSG_LEN,
                                                   (uint8_t *)&acp_cmd,
                                                   0,
                                                   NULL,
                                                   NULL);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Initiator instance removed" APP_LOG_NL, conn_handle);

      // Restart scanning for new reflector connections
      (void)ble_peer_manager_central_create_connection();
      app_log_info(APP_PREFIX "Scanning restarted for new reflector connections..." APP_LOG_NL);

      cs_host_state.num_reflector_connections--;
      app_log_info(APP_PREFIX "Initiator instances in use: %u/%u" APP_LOG_NL,
                   cs_host_state.num_reflector_connections,
                   cs_host_config.max_initiator_instances);
      break;
    }
  }

  // Check if the connection handle is an initiator connection - if yes remove the corresponding reflector instance
  for (uint32_t i = 0u; i < cs_host_config.max_reflector_instances; i++) {
    if (cs_host_state.initiator_conn_handles[i] == conn_handle) {
      cs_host_state.initiator_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;
      app_log_info(APP_INSTANCE_PREFIX "Initiator connection closed" APP_LOG_NL, conn_handle);

      // Call the ACP API to remove the reflector instance for the connection handle
      sl_status_t sc;
      cs_acp_cmd_t msg;

      msg.cmd_id = CS_ACP_CMD_REFLECTOR_ACTION;
      msg.data.reflector_action_data.connection_id = conn_handle;
      msg.data.reflector_action_data.reflector_action = CS_ACP_ACTION_DELETE_REFLECTOR;
      sc = sl_bt_user_cs_service_message_to_target(REFLECTOR_ACTION_MSG_LEN,
                                                   (uint8_t *)&msg,
                                                   0,
                                                   NULL,
                                                   NULL);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Reflector instance removed" APP_LOG_NL, conn_handle);

      // Restart advertising for new initiator connections if we were at the limit
      if (cs_host_state.num_initiator_connections == cs_host_config.max_reflector_instances) {
        sl_status_t sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
        app_assert_status(sc);
        app_log_info(APP_PREFIX "Advertising restarted for new initiator connections..." APP_LOG_NL);
      }

      cs_host_state.num_initiator_connections--;
      app_log_info(APP_PREFIX "Reflector instances in use: %u/%u" APP_LOG_NL,
                   cs_host_state.num_initiator_connections,
                   cs_host_config.max_reflector_instances);
      break;
    }
  }
}

void ble_peer_manager_on_event(ble_peer_manager_evt_type_t *event)
{
  switch (event->evt_id) {
    case BLE_PEER_MANAGER_ON_CONN_OPENED_CENTRAL:
      app_log_info(APP_INSTANCE_PREFIX "Connection opened as central with a CS Reflector" APP_LOG_NL, event->connection_id);
      handle_connection_opened_with_reflector(event->connection_id);
      break;

    case BLE_PEER_MANAGER_ON_CONN_OPENED_PERIPHERAL:
      app_log_info(APP_INSTANCE_PREFIX "Connection opened as peripheral with a CS Initiator" APP_LOG_NL, event->connection_id);
      handle_connection_opened_with_initiator(event->connection_id);
      break;

    case BLE_PEER_MANAGER_ON_CONN_CLOSED:
      app_log_info(APP_INSTANCE_PREFIX "Connection closed" APP_LOG_NL, event->connection_id);
      handle_connection_closed(event->connection_id);
      break;

    case BLE_PEER_MANAGER_ON_ADV_STOPPED:
      app_log_info(APP_INSTANCE_PREFIX "Advertisement stopped" APP_LOG_NL, event->connection_id);
      break;

    case BLE_PEER_MANAGER_ERROR:
      app_log_info(APP_INSTANCE_PREFIX "Error" APP_LOG_NL, event->connection_id);
      break;

    default:
      app_log_info(APP_INSTANCE_PREFIX "Unhandled Peer Manager event (%u)" APP_LOG_NL, event->connection_id, event->evt_id);
      break;
  }
}

/******************************************************************************
 * Extract and display measurement results
 *****************************************************************************/
void cs_on_result(cs_result_t *result, void *user_data)
{
  (void)user_data;

  app_log_info("---" APP_LOG_NL);
  app_log_info(APP_INSTANCE_PREFIX "Measurement result: %u mm" APP_LOG_NL,
               result->connection,
               (uint32_t)(result->distance * 1000.f));
  app_log_info(APP_INSTANCE_PREFIX "Measurement likeliness: %f" APP_LOG_NL,
               result->connection,
               result->likeliness);
  app_log_info(APP_INSTANCE_PREFIX "RSSI distance: %u mm" APP_LOG_NL,
               result->connection,
               (uint32_t)(result->rssi_distance * 1000.f));
  app_log_info(APP_INSTANCE_PREFIX "CS bit error rate: %u" APP_LOG_NL,
               result->connection,
               result->cs_bit_error_rate);
}

/******************************************************************************
 * Extract and display intermediate results between measurement results
 * Note: only called when stationary object tracking used
 *****************************************************************************/
void cs_on_intermediate_result(const cs_intermediate_result_t *intermediate_result,
                               const void *user_data)
{
  (void)user_data;

  app_log_info("---" APP_LOG_NL);
  uint32_t percentage_whole = (uint32_t)intermediate_result->progress_percentage;
  uint32_t percentage_frac =
    (uint32_t)((intermediate_result->progress_percentage - (float)percentage_whole) * 100.0f);

  app_log_info(APP_INSTANCE_PREFIX "Estimation in progress: %lu.%02lu%%" APP_LOG_NL,
               intermediate_result->connection,
               (unsigned long)percentage_whole,
               (unsigned long)percentage_frac);
}

/******************************************************************************
 * CS error handler
 *****************************************************************************/
void cs_on_error(uint8_t conn_handle, cs_error_event_t err_evt, sl_status_t sc)
{
  switch (err_evt) {
    // Assert
    case CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED:
    case CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA:
      app_assert(false,
                 APP_INSTANCE_PREFIX "Unrecoverable CS procedure error happened!"
                                     "[E: 0x%x sc: 0x%x]" APP_LOG_NL,
                 conn_handle,
                 err_evt,
                 sc);
      break;
    // Discard
    case CS_ERROR_EVENT_RTL_PROCESS_ERROR:
      app_log_error(APP_INSTANCE_PREFIX "RTL processing error happened!"
                                        "[E: 0x%x sc: 0x%x]" APP_LOG_NL,
                    conn_handle,
                    err_evt,
                    sc);
      break;
    // Close connection
    default:
      app_log_error(APP_INSTANCE_PREFIX "Error happened! Closing connection"
                                        "[E: 0x%x sc: 0x%x]" APP_LOG_NL,
                    conn_handle,
                    err_evt,
                    sc);
      // Common errors
      if (err_evt == CS_ERROR_EVENT_TIMER_ELAPSED) {
        app_log_error(APP_INSTANCE_PREFIX "Operation timeout." APP_LOG_NL, conn_handle);
      } else if (err_evt == CS_ERROR_EVENT_INITIATOR_FAILED_TO_INCREASE_SECURITY) {
        app_log_error(APP_INSTANCE_PREFIX "Security level increase failed." APP_LOG_NL, conn_handle);
      }
      // Close the connection
      (void)ble_peer_manager_central_close_connection(conn_handle);
      break;
  }
}

void stop_procedure(void)
{
  cs_acp_cmd_t acp_cmd;
  sl_status_t sc;
  uint8_t conn_handle;

  // Close all reflector connections, and delete initiator instances on the NCP
  for (uint32_t i = 0u; i < cs_host_config.max_initiator_instances; i++) {
    conn_handle = cs_host_state.reflector_conn_handles[i];
    if (conn_handle != SL_BT_INVALID_CONNECTION_HANDLE) {
      app_log_info(APP_PREFIX "Removing Initiator instance; connection_handle='%u'" APP_LOG_NL, conn_handle);
      cs_host_state.reflector_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;

      // Call the ACP API to remove the initiator instance for the connection handle

      acp_cmd.cmd_id = CS_ACP_CMD_INITIATOR_ACTION;
      acp_cmd.data.initiator_action_data.connection_id = conn_handle;
      acp_cmd.data.initiator_action_data.initiator_action = CS_ACP_ACTION_DELETE_INITIATOR;
      sc = sl_bt_user_cs_service_message_to_target(INITATOR_ACTION_MSG_LEN,
                                                   (uint8_t *)&acp_cmd,
                                                   0,
                                                   NULL,
                                                   NULL);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Initiator instance removed" APP_LOG_NL, conn_handle);

      sc = ble_peer_manager_central_close_connection(conn_handle);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Reflector connection closed" APP_LOG_NL, conn_handle);

      cs_host_state.num_reflector_connections--;
      app_log_info(APP_PREFIX "Initiator instances in use: %u/%u" APP_LOG_NL,
                   cs_host_state.num_reflector_connections,
                   cs_host_config.max_initiator_instances);
    }
  }

  // Close all initiator connections, and delete reflector instances on the NCP
  for (uint32_t i = 0u; i < cs_host_config.max_reflector_instances; i++) {
    conn_handle = cs_host_state.initiator_conn_handles[i];
    if (conn_handle != SL_BT_INVALID_CONNECTION_HANDLE) {
      cs_host_state.initiator_conn_handles[i] = SL_BT_INVALID_CONNECTION_HANDLE;

      // Call the ACP API to remove the initiator instance for the connection handle
      acp_cmd.cmd_id = CS_ACP_CMD_REFLECTOR_ACTION;
      acp_cmd.data.reflector_action_data.connection_id = conn_handle;
      acp_cmd.data.reflector_action_data.reflector_action = CS_ACP_ACTION_DELETE_REFLECTOR;
      sc = sl_bt_user_cs_service_message_to_target(REFLECTOR_ACTION_MSG_LEN,
                                                   (uint8_t *)&acp_cmd,
                                                   0,
                                                   NULL,
                                                   NULL);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Reflector instance removed" APP_LOG_NL, conn_handle);

      sc = ble_peer_manager_peripheral_close_connection(conn_handle);
      app_assert_status(sc);
      app_log_info(APP_INSTANCE_PREFIX "Initiator connection closed" APP_LOG_NL, conn_handle);

      cs_host_state.num_initiator_connections--;
      app_log_info(APP_PREFIX "Reflector instances in use: %u/%u" APP_LOG_NL,
                   cs_host_state.num_initiator_connections,
                   cs_host_config.max_reflector_instances);
    }
  }
}
