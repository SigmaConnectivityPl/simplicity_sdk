/***************************************************************************//**
 * @file
 * @brief CS Reflector example application logic
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
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app_log.h"
#include "app.h"
#include "ble_peer_manager_connections.h"
#include "ble_peer_manager_peripheral.h"

#include "cs_reflector.h"
#include "cs_reflector_config.h"
#include "cs_antenna.h"

#define APP_PREFIX                  "[APP] "
#define INSTANCE_PREFIX             "[%u] "
#define APP_INSTANCE_PREFIX         APP_PREFIX INSTANCE_PREFIX

static cs_reflector_config_t cs_reflector_config = {
  .max_tx_power_dbm = CS_REFLECTOR_MAX_TX_POWER_DBM
};

/**************************************************************************//**
 * Application Init
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  ble_peer_manager_peripheral_init();

  app_log_filter_threshold_set(APP_LOG_LEVEL_INFO);

  app_log_info(APP_LOG_NL);
  app_log_info("+-[CS Reflector by Silicon Labs]------------------------+" APP_LOG_NL);
  app_log_info("+-------------------------------------------------------+" APP_LOG_NL);
  app_log_info(APP_PREFIX "Maximum concurrent connections: %u" APP_LOG_NL, CS_REFLECTOR_MAX_CONNECTIONS);
  app_log_info(APP_PREFIX "Channel Sounding event buffer size: %u" APP_LOG_NL, CS_REFLECTOR_CS_EVENT_BUF_SIZE);
  app_log_info(APP_PREFIX "Default minimum transmit power: %d dBm" APP_LOG_NL, CS_REFLECTOR_MIN_TX_POWER_DBM);
  app_log_info(APP_PREFIX "Default maximum transmit power: %d dBm" APP_LOG_NL, CS_REFLECTOR_MAX_TX_POWER_DBM);

  app_log_info(APP_PREFIX "Wire%s antenna offset will be used." APP_LOG_NL,
               CS_REFLECTOR_ANTENNA_OFFSET ? "d" : "less");

  app_log_info("+-------------------------------------------------------+" APP_LOG_NL APP_LOG_NL);

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
    {
      sl_status_t sc;
      bd_addr address;
      uint8_t address_type;
      int16_t min_tx_power_x10 = CS_REFLECTOR_MIN_TX_POWER_DBM * 10;
      int16_t max_tx_power_x10 = CS_REFLECTOR_MAX_TX_POWER_DBM * 10;
      sc = sl_bt_system_set_tx_power(min_tx_power_x10,
                                     max_tx_power_x10,
                                     &min_tx_power_x10,
                                     &max_tx_power_x10);
      app_assert_status(sc);
      app_log_info(APP_PREFIX "Set minimum transmit power to: %d dBm" APP_LOG_NL, min_tx_power_x10 / 10);
      app_log_info(APP_PREFIX "Set maximum transmit power to: %d dBm" APP_LOG_NL, max_tx_power_x10 / 10);
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);
      // Print the Bluetooth address
      app_log_info(APP_PREFIX "Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);

      // Set antenna offset
      sc = cs_antenna_configure(CS_REFLECTOR_ANTENNA_OFFSET);
      app_assert_status(sc);

      // Start advertising for initiator connections
      if (CS_REFLECTOR_MAX_CONNECTIONS > 0) {
        sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
        app_assert_status(sc);
        app_log_info(APP_PREFIX "Advertising started for initiator connections..." APP_LOG_NL);
      }
    }
    break;

    // -------------------------------
    // This event indicates that the BT stack buffer resources were exhausted
    case sl_bt_evt_system_resource_exhausted_id:
    {
      uint8_t num_buffers_discarded = evt->data.evt_system_resource_exhausted.num_buffers_discarded;
      uint8_t num_buffer_allocation_failures = evt->data.evt_system_resource_exhausted.num_buffer_allocation_failures;
      uint8_t num_heap_allocation_failures = evt->data.evt_system_resource_exhausted.num_heap_allocation_failures;
      app_log_error(APP_PREFIX "BT stack buffers exhausted, data loss may have occurred! "
                               "buf_discarded='%u' buf_alloc_fail='%u' heap_alloc_fail='%u'" APP_LOG_NL,
                    num_buffers_discarded,
                    num_buffer_allocation_failures,
                    num_heap_allocation_failures);
    }
    break;

    // -------------------------------
    // This event is received when a BT stack system error occurs
    case sl_bt_evt_system_error_id:
      app_log_error(APP_PREFIX "System error occurred; reason='0x%02x'" APP_LOG_NL, evt->data.evt_system_error.reason);
      break;

    // -------------------------------
    // This event is received when a bonding procedure fails
    case sl_bt_evt_sm_bonding_failed_id:
    {
      uint8_t conn_handle = evt->data.evt_sm_bonding_failed.connection;
      app_log_error(APP_INSTANCE_PREFIX "Bonding failed; reason='0x%02x'" APP_LOG_NL,
                    conn_handle,
                    evt->data.evt_sm_bonding_failed.reason);
      sl_status_t sc = sl_bt_connection_close(conn_handle);
      app_assert_status(sc);
    }
    break;

    // -------------------------------
    // This event indicates that a MTU exchange has finished
    case sl_bt_evt_gatt_mtu_exchanged_id:
    {
      uint8_t conn_handle = evt->data.evt_gatt_mtu_exchanged.connection;
      app_log_info(APP_INSTANCE_PREFIX "MTU exchange completed" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that the connection parameters were changed
    case sl_bt_evt_connection_parameters_id:
    {
      uint8_t conn_handle = evt->data.evt_connection_parameters.connection;
      app_log_info(APP_INSTANCE_PREFIX "Connection parameters changed" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that a PHY update procedure has finished
    case sl_bt_evt_connection_phy_status_id:
    {
      uint8_t conn_handle = evt->data.evt_connection_phy_status.connection;
      app_log_info(APP_INSTANCE_PREFIX "PHY update procedure completed" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that the features supported by the remote device's LL were updated
    case sl_bt_evt_connection_remote_used_features_id:
    {
      uint8_t conn_handle = evt->data.evt_connection_remote_used_features.connection;
      app_log_info(APP_INSTANCE_PREFIX "Remote LL supported features updated" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that the maximum Rx/Tx data length was changed
    case sl_bt_evt_connection_data_length_id:
    {
      uint8_t conn_handle = evt->data.evt_connection_data_length.connection;
      app_log_info(APP_INSTANCE_PREFIX "Maximum payload length changed" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that a bonding procedure has successfully finished
    case sl_bt_evt_sm_bonded_id:
    {
      uint8_t conn_handle = evt->data.evt_sm_bonded.connection;
      app_log_info(APP_INSTANCE_PREFIX "Bonding procedure successfully completed" APP_LOG_NL, conn_handle);
    }
    break;

    // -------------------------------
    // This event indicates that the radio transmit power was changed
    case sl_bt_evt_connection_tx_power_id:
    {
      uint8_t conn_handle = evt->data.evt_connection_tx_power.connection;
      int8_t tx_power = evt->data.evt_connection_tx_power.power_level;
      app_log_info(APP_INSTANCE_PREFIX "Transmit power changed; tx_power='%d'" APP_LOG_NL, conn_handle, tx_power);
    }
    break;

    // This event indicates that a new connection was opened
    // This case is here to suppress the default event handler
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    // This case is here to suppress the default event handler
    case sl_bt_evt_connection_closed_id:
      break;

    // -------------------------------
    // This event is received when an advertisement times out
    // This case is here to suppress the default event handler
    case sl_bt_evt_advertiser_timeout_id:
      break;

    // -------------------------------
    // This event indicates that a new CS result is available
    // This case is here to suppress the default event handler
    case sl_bt_evt_cs_result_id:
      break;

    // -------------------------------
    // Default event handler
    default:
      app_log_info(APP_PREFIX "BLE event: 0x%02lx" APP_LOG_NL, SL_BT_MSG_ID(evt->header));
      break;
  }
}

void handle_connection_opened_with_initiator(uint8_t conn_handle)
{
  sl_status_t sc;
  // Create a new reflector instance for the connection handle
  sc = cs_reflector_create(conn_handle, &cs_reflector_config);
  if (sc != SL_STATUS_OK) {
    app_log_error(APP_INSTANCE_PREFIX "Failed to create reflector instance'" APP_LOG_NL, conn_handle);
    ble_peer_manager_peripheral_close_connection(conn_handle);
    return;
  }

  // Advertise for new initiator connections if we have room for more
  if (cs_reflector_get_active_instance_count() < CS_REFLECTOR_MAX_CONNECTIONS) {
    sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
    app_assert_status(sc);
    app_log_info(APP_PREFIX "Advertising restarted for new initiator connections..." APP_LOG_NL);
  }
}

void handle_connection_closed(uint8_t conn_handle)
{
  sl_status_t sc;
  bool advertisement_should_be_restarted = false;
  uint8_t reflector_count = cs_reflector_get_active_instance_count();
  // If we are at the maximum capacity - it means that the advertisement is not running
  // Restart advertising for new initiator connections if we were at the limit
  if (reflector_count == CS_REFLECTOR_MAX_CONNECTIONS) {
    advertisement_should_be_restarted = true;
  }

  // Remove the reflector instance for the connection handle
  sc = cs_reflector_delete(conn_handle);
  app_assert_status_f(sc, "Failed to delete reflector instance");

  // Restart advertising if needed
  if (advertisement_should_be_restarted) {
    sc = ble_peer_manager_peripheral_start_advertising(SL_BT_INVALID_ADVERTISING_SET_HANDLE);
    app_assert_status(sc);
    app_log_info(APP_PREFIX "Advertising restarted for new initiator connections..." APP_LOG_NL);
  }
}

void ble_peer_manager_on_event_reflector(const ble_peer_manager_evt_type_t *event)
{
  switch (event->evt_id) {
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
