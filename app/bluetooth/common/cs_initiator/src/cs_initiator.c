/***************************************************************************//**
 * @file
 * @brief CS Initiator logic.
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

#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <limits.h>

#include "sl_bt_api.h"
#include "sl_component_catalog.h"
#include "sl_common.h"
#include "sl_sleeptimer.h"
#include "cs_initiator.h"
#include "cs_initiator_configurator.h"
#include "cs_initiator_config.h"
#include "cs_ras.h"
#include "app_timer.h"

#include "sl_rtl_clib_api.h"

#ifdef SL_CATALOG_APP_LOG_PRESENT
#include "app_log.h"
#endif // SL_CATALOG_APP_LOG_PRESENT

// -----------------------------------------------------------------------------
// Macros
// Component logging
#if defined(SL_CATALOG_APP_LOG_PRESENT) && CS_INITIATOR_LOG
#define LOG_PREFIX                  CS_INITIATOR_LOG_PREFIX " "
#define LOG_NL                      APP_LOG_NL
#define INSTANCE_PREFIX             "[%u] "
#define initiator_log_debug(...)    app_log_debug(LOG_PREFIX  __VA_ARGS__)
#define initiator_log_info(...)     app_log_info(LOG_PREFIX  __VA_ARGS__)
#define initiator_log_warning(...)  app_log_warning(LOG_PREFIX  __VA_ARGS__)
#define initiator_log_error(...)    app_log_error(LOG_PREFIX  __VA_ARGS__)
#define initiator_log_critical(...) app_log_critical(LOG_PREFIX  __VA_ARGS__)
#else
#define LOG_NL
#define INSTANCE_PREFIX
#define initiator_log_debug(...)
#define initiator_log_info(...)
#define initiator_log_warning(...)
#define initiator_log_error(...)
#define initiator_log_critical(...)
#endif // defined(SL_CATALOG_APP_LOG_PRESENT) && CS_INITIATOR_LOG

// This configuration is for future use only
#define CONFIG_RAS_REQUEST_PERIODIC_NOTIFICATION_MODE true

#define CS_CHANNEL_MAP_MIN_CHANNELS    15
#define CS_CHANNEL_MAP_MAX_CHANNELS    76
#define CS_MAX_PROCEDURE_ENABLE_RETRY  3

// -----------------------------------------------------------------------------
// Enums, structs, typedefs

typedef enum {
  CS_TRANSPORT_INIT_STATE = 0,
  CS_TRANSPORT_START_DISCOVERING_RAS_CHARACTERISTICS,
  CS_TRANSPORT_WAIT_FOR_RAS_CHARACTERISTICS_DISCOVERY_COMPLETE,
  CS_TRANSPORT_SUBSCRIBE_RAS_SUBEVENT_RANGING_DATA,
  CS_TRANSPORT_SUBSCRIBE_RAS_CONTROL_POINT,
  CS_TRANSPORT_STATE_IDLE
} action_t;

typedef enum {
  INITIATOR_STATE_UNINITIALIZED = 0,
  INITIATOR_STATE_INIT,
  INITIATOR_STATE_START_PROCEDURE,
  INITIATOR_STATE_WAIT_PROCEDURE_ENABLE_COMPLETE,
  INITIATOR_STATE_WAIT_PROCEDURE_DISABLE_COMPLETE,
  INITIATOR_STATE_IN_PROCEDURE,
  INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_COMPLETE,
  INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_ABORTED,
  INITIATOR_STATE_WAIT_INITIATOR_PROCEDURE_ABORTED,
  INITIATOR_STATE_DELETE,
  INITIATOR_STATE_ERROR
} initiator_state_t;

typedef struct {
  sl_rtl_cs_subevent_data subevent_data[CS_INITIATOR_MAX_SUBEVENT_PER_PROC];
  uint8_t step_data[CS_INITIATOR_MAX_STEP_DATA_LEN];
  uint16_t step_index;
  uint16_t subevent_index;
  uint16_t procedure_counter;
  bool ready;
} cs_data_t;

// CS Initiator measurement buffer
typedef struct {
  cs_data_t initiator_data;
  cs_data_t reflector_data;
  sl_rtl_cs_procedure cs_procedure;
  rtl_config_t rtl_config;
  cs_initiator_config_t config;
  uint16_t current_cs_subevent_index;  // Associated with ongoing CS measurement
  uint16_t current_procedure_index;    // Associated with ongoing CS measurement
  uint8_t conn_handle;
  uint32_t ras_service_handle;
  struct {
    uint16_t ras_control_point;
    uint16_t ras_procedure_enable_data;
    uint16_t ras_subevent_ranging_data;
  } characteristic_handle;
  struct {
    action_t action;
    struct {
      bool ras_control_point;
      bool ras_procedure_enable_data;
      bool ras_subevent_ranging_data;
      bool ras_all;
    } characteristic_found;
  } discovery_state;
  bool cs_security_enabled;
  sl_rtl_cs_libitem rtl_handle;
  cs_result_cb_t result_cb;
  cs_intermediate_result_cb_t intermediate_result_cb;
  cs_error_cb_t error_cb;
  uint32_t procedure_start_time_ms;
  uint32_t procedure_stop_time_ms;
  uint32_t log_error_counter;
  app_timer_t timer_handle;
  bool error_timer_started;
  bool error_timer_elapsed;
  initiator_state_t initiator_state;
  cs_procedure_state_t initiator_procedure_state;
  cs_procedure_state_t reflector_procedure_state;
  uint8_t procedure_enable_retry_counter;
} cs_initiator_t;

// -----------------------------------------------------------------------------
// Static function declarations

static cs_initiator_t *cs_initiator_get_instance(const uint8_t conn_handle);
static cs_initiator_t *cs_initiator_get_free_slot();
static void set_procedure_initial_state(const uint8_t         conn_handle,
                                        cs_initiator_config_t *initiator_config,
                                        sl_rtl_cs_procedure   *cs_procedure);
static bool gatt_procedure_complete_event(const uint8_t conn_handle,
                                          sl_bt_msg_t    *evt);
static sl_status_t initiator_finalize_cleanup(cs_initiator_t *initiator);
static initiator_state_t initiator_stop_procedure_on_invalid_state(cs_initiator_t *initiator);
static void handle_procedure_enable_completed_event_disable(cs_initiator_t *initiator);
static bool check_characteristic_uuid(const uint8_t conn_handle,
                                      sl_bt_msg_t *evt);
static cs_procedure_state_t extract_device_cs_data(const uint8_t conn_handle,
                                                   cs_data_t *device_data,
                                                   sl_rtl_cs_subevent_data *rtl_subevent_data,
                                                   uint8_t *rtl_subevent_data_count,
                                                   sl_bt_evt_cs_result_t *evt);
static cs_procedure_state_t extract_cs_result_data(cs_initiator_t *initiator,
                                                   bool initiator_part,
                                                   sl_bt_evt_cs_result_t *evt);
static void show_rtl_api_call_result(const uint8_t conn_handle,
                                     enum sl_rtl_error_code err_code);
static void calculate_distance(const uint8_t conn_handle);
static uint32_t get_num_tones_from_channel_map(const uint8_t *ch_map,
                                               const uint32_t ch_map_len);
static void reset_subevent_data(const uint8_t conn_handle);
static void on_error(const uint8_t conn_handle,
                     cs_error_event_t evt,
                     sl_status_t      sc);
static enum sl_rtl_error_code rtl_library_init(const uint8_t conn_handle,
                                               sl_rtl_cs_libitem *handle,
                                               rtl_config_t      *config,
                                               const uint8_t     cs_mode);
static void init_cs_configuration(const uint8_t conn_handle);
static void start_error_timer(const uint8_t conn_handle);
static void stop_error_timer(const uint8_t conn_handle);
static void error_timer_cb(app_timer_t *handle, void *data);

// -----------------------------------------------------------------------------
// State machine forward declaration
static sl_status_t initiator_state_machine_event_handler(uint8_t                    conn_handle,
                                                         state_machine_event_t      event,
                                                         state_machine_event_data_t *data);
static sl_status_t state_any_on_error(cs_initiator_t             *initiator,
                                      state_machine_event_data_t *data);
static sl_status_t state_any_on_delete_instance(cs_initiator_t             *initiator,
                                                state_machine_event_data_t *data);
static sl_status_t state_uninitialized_on_init_started(cs_initiator_t             *initiator,
                                                       state_machine_event_data_t *data);
static sl_status_t state_init_on_start_init_completed(cs_initiator_t             *initiator,
                                                      state_machine_event_data_t *data);
static sl_status_t state_start_procedure_on_start_procedure(cs_initiator_t             *initiator,
                                                            state_machine_event_data_t *data);
static sl_status_t state_wait_procedure_enable_complete_on_enable_completed(cs_initiator_t             *initiator,
                                                                            state_machine_event_data_t *data);
static sl_status_t state_in_procedure_on_cs_result(cs_initiator_t             *initiator,
                                                   state_machine_event_data_t *data);
static sl_status_t state_wait_reflector_procedure_complete_on_cs_result(cs_initiator_t             *initiator,
                                                                        state_machine_event_data_t *data);
static sl_status_t state_wait_reflector_procedure_aborted_on_cs_result(cs_initiator_t             *initiator,
                                                                       state_machine_event_data_t *data);
static sl_status_t state_wait_initiator_procedure_aborted_on_cs_result(cs_initiator_t             *initiator,
                                                                       state_machine_event_data_t *data);
static sl_status_t state_wait_procedure_disable_on_procedure_enable_completed(cs_initiator_t             *initiator,
                                                                              state_machine_event_data_t *data);
static sl_status_t state_delete_on_procedure_enable_completed(cs_initiator_t             *initiator,
                                                              state_machine_event_data_t *data);

// -----------------------------------------------------------------------------
// Static variables
static cs_initiator_t cs_initiator_instances[CS_INITIATOR_MAX_CONNECTIONS];

static const uint8_t ras_service_uuid[] = RAS_SERVICE_UUID;
static const uint8_t ras_control_point_characteristic_uuid[] =
  RAS_CONTROL_POINT_CHARACTERISTIC_UUID;

static const uint8_t ras_procedure_enable_data_characteristic_uuid[] =
  RAS_PROCEDURE_ENABLE_DATA_CHARACTERISTIC_UUID;

static const uint8_t ras_se_ranging_data_characteristic_uuid[] =
  RAS_SE_RANGING_DATA_CHARACTERISTIC_UUID;

// -----------------------------------------------------------------------------
// Private function definitions

/******************************************************************************
 * Get pointer to initiator instance based on the connection handle.
 *****************************************************************************/
static cs_initiator_t *cs_initiator_get_instance(const uint8_t conn_handle)
{
  for (uint8_t i = 0u; i < CS_INITIATOR_MAX_CONNECTIONS; i++) {
    if (cs_initiator_instances[i].conn_handle == conn_handle) {
      return &cs_initiator_instances[i];
    }
  }
  initiator_log_error("No matching instance found for connection handle %u!" LOG_NL,
                      conn_handle);
  return NULL;
}

/******************************************************************************
 * Get free slot for new initiator instance.
 * @return pointer to the free slot.
 * @return NULL if no free slot is available.
 *****************************************************************************/
static cs_initiator_t *cs_initiator_get_free_slot()
{
  for (uint8_t i = 0u; i < CS_INITIATOR_MAX_CONNECTIONS; i++) {
    if (cs_initiator_instances[i].conn_handle == SL_BT_INVALID_CONNECTION_HANDLE) {
      initiator_log_debug("free slot found." LOG_NL);
      return &cs_initiator_instances[i];
    }
  }
  initiator_log_error("no free slot!" LOG_NL);
  return NULL;
}

/******************************************************************************
 * CS Initiator set initial config.
 *****************************************************************************/
static void set_procedure_initial_state(const uint8_t         conn_handle,
                                        cs_initiator_config_t *initiator_config,
                                        sl_rtl_cs_procedure   *cs_procedure)
{
  initiator_log_debug(INSTANCE_PREFIX "CS procedure - configuration - begin" LOG_NL,
                      conn_handle);
  cs_procedure->cs_config.ch3c_jump = initiator_config->ch3c_jump;
  cs_procedure->cs_config.ch3c_shape = initiator_config->ch3c_shape;
  initiator_log_debug(INSTANCE_PREFIX "ch3c_jump=%u, ch3c_shape=%u" LOG_NL,
                      conn_handle,
                      cs_procedure->cs_config.ch3c_jump,
                      cs_procedure->cs_config.ch3c_shape);

  cs_procedure->cs_config.channel_map_repetition =
    initiator_config->channel_map_repetition;
  cs_procedure->cs_config.cs_sync_phy = (cs_phy_t)initiator_config->phy;
  initiator_log_debug(INSTANCE_PREFIX "channel_map_repetition=%u, cs_sync_phy=%u" LOG_NL,
                      conn_handle,
                      cs_procedure->cs_config.channel_map_repetition,
                      cs_procedure->cs_config.cs_sync_phy);

  cs_procedure->cs_config.main_mode_repetition =
    initiator_config->main_mode_repetition;
  cs_procedure->cs_config.procedure_count = 1;
  initiator_log_debug(INSTANCE_PREFIX "main_mode_repetition=%lu,  procedure_count=%u" LOG_NL,
                      conn_handle,
                      (unsigned long)cs_procedure->cs_config.main_mode_repetition,
                      cs_procedure->cs_config.procedure_count);

  cs_procedure->cs_config.rtt_type = initiator_config->rtt_type;
  cs_procedure->cs_config.subevents_per_event = 1;
  cs_procedure->cs_config.num_antenna_paths = 1;
  initiator_log_debug(INSTANCE_PREFIX "rtt_type=%u,  subevents_per_event=%u,"
                                      " num_antenna_paths=%u" LOG_NL,
                      conn_handle,
                      cs_procedure->cs_config.rtt_type,
                      cs_procedure->cs_config.subevents_per_event,
                      cs_procedure->cs_config.num_antenna_paths);

  initiator_log_debug(INSTANCE_PREFIX "CS procedure - configuration - end" LOG_NL,
                      conn_handle);
}

/******************************************************************************
 * Error timer callback.
 *****************************************************************************/
static void error_timer_cb(app_timer_t *handle, void *data)
{
  cs_initiator_t *initiator = (cs_initiator_t *)data;
  if (handle == &initiator->timer_handle) {
    initiator->error_timer_started = false;
    initiator->error_timer_elapsed = true;
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_TIMER_ELAPSED,
             SL_STATUS_TIMEOUT);
  }
}

static void procedure_timer_cb(app_timer_t *handle, void *data)
{
  cs_initiator_t *initiator = (cs_initiator_t *)data;
  if (handle == &initiator->timer_handle) {
    initiator->error_timer_started = false;
    initiator->error_timer_elapsed = true;
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_TIMER_ELAPSED,
             SL_STATUS_TIMEOUT);
  }
}

/******************************************************************************
 * Initiator finalize cleanup. Remove the configuration and deinit RTL lib.
 * This function is called after the procedure was stopped.
 * @param[in] initiator pointer to the initiator instance.
 * @return SL_STATUS_OK if cleanup was successful.
 * @return SL_STATUS_FAIL if cleanup failed.
 *****************************************************************************/
static sl_status_t initiator_finalize_cleanup(cs_initiator_t *initiator)
{
  enum sl_rtl_error_code rtl_err;
  sl_status_t sc = SL_STATUS_OK;
  (void)sl_bt_cs_remove_config(initiator->conn_handle, initiator->config.config_id);

  if (initiator->rtl_handle != NULL) {
    rtl_err = sl_rtl_cs_deinit(&initiator->rtl_handle);
    if (rtl_err != SL_RTL_ERROR_SUCCESS) {
      initiator_log_error(INSTANCE_PREFIX "Failed to deinit RTL lib! [err: 0x%02x]" LOG_NL,
                          initiator->conn_handle,
                          rtl_err);
      return SL_STATUS_FAIL;
    }
  }

  initiator_log_debug(INSTANCE_PREFIX "deleting instance" LOG_NL,
                      initiator->conn_handle);

  memset(initiator, 0, sizeof(cs_initiator_t));
  initiator_log_info(INSTANCE_PREFIX "instance deleted" LOG_NL, initiator->conn_handle);
  initiator->conn_handle = SL_BT_INVALID_CONNECTION_HANDLE;
  return sc;
}

static initiator_state_t initiator_stop_procedure_on_invalid_state(cs_initiator_t *initiator)
{
  sl_status_t sc;

  sc = sl_bt_cs_procedure_enable(initiator->conn_handle,
                                 sl_bt_cs_procedure_state_disabled,
                                 initiator->config.config_id);
  if (sc == SL_STATUS_OK) {
    initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_PROCEDURE_DISABLE_COMPLETE" LOG_NL,
                       initiator->conn_handle);
    return INITIATOR_STATE_WAIT_PROCEDURE_DISABLE_COMPLETE;
  } else if (sc == SL_STATUS_INVALID_HANDLE) {
    // Procedure is already stopped by stack
    app_timer_stop(&initiator->timer_handle);
    initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                       initiator->conn_handle);
    return INITIATOR_STATE_START_PROCEDURE;
  } else {
    // all other errors creates an error state
    return INITIATOR_STATE_ERROR;
  }
}

/******************************************************************************
 * Initiator state machine
 *****************************************************************************/
static sl_status_t initiator_state_machine_event_handler(uint8_t                    conn_handle,
                                                         state_machine_event_t      event,
                                                         state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;

  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL, conn_handle);
    on_error(conn_handle,
             CS_ERROR_EVENT_STATE_MACHINE_FAILED,
             SL_STATUS_NULL_POINTER);
    return SL_STATUS_NULL_POINTER;
  }

  if (event == INITIATOR_EVT_ERROR) {
    sc = state_any_on_error(initiator, data);
  }

  if (event == INITIATOR_EVT_DELETE_INSTANCE) {
    sc = state_any_on_delete_instance(initiator, data);
    return sc;
  }

  switch (initiator->initiator_state) {
    case INITIATOR_STATE_UNINITIALIZED:
      if (event == INITIATOR_EVT_INIT_STARTED) {
        sc = state_uninitialized_on_init_started(initiator, data);
      }
      break;

    case INITIATOR_STATE_INIT:
      if (event == INITIATOR_EVT_INIT_COMPLETED) {
        sc = state_init_on_start_init_completed(initiator, data);
      }
      break;

    case INITIATOR_STATE_START_PROCEDURE:
      if (event == INITIATOR_EVT_START_PROCEDURE) {
        sc = state_start_procedure_on_start_procedure(initiator, data);
      }
      break;

    case INITIATOR_STATE_WAIT_PROCEDURE_ENABLE_COMPLETE:
      if (event == INITIATOR_EVT_PROCEDURE_ENABLE_COMPLETED) {
        sc = state_wait_procedure_enable_complete_on_enable_completed(initiator,
                                                                      data);
      }
      break;

    case INITIATOR_STATE_IN_PROCEDURE:
      if (event == INITIATOR_EVT_CS_RESULT) {
        sc = state_in_procedure_on_cs_result(initiator, data);
      }
      break;

    case INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_COMPLETE:
      if (event == INITIATOR_EVT_CS_RESULT) {
        sc = state_wait_reflector_procedure_complete_on_cs_result(initiator, data);
      }
      break;

    case INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_ABORTED:
      if (event == INITIATOR_EVT_CS_RESULT) {
        sc = state_wait_reflector_procedure_aborted_on_cs_result(initiator,
                                                                 data);
      }
      break;
    case INITIATOR_STATE_WAIT_INITIATOR_PROCEDURE_ABORTED:
      if (event == INITIATOR_EVT_CS_RESULT) {
        sc = state_wait_initiator_procedure_aborted_on_cs_result(initiator,
                                                                 data);
      }
      break;

    case INITIATOR_STATE_WAIT_PROCEDURE_DISABLE_COMPLETE:
      if (event == INITIATOR_EVT_PROCEDURE_ENABLE_COMPLETED) {
        sc = state_wait_procedure_disable_on_procedure_enable_completed(initiator,
                                                                        data);
      }
      break;

    case INITIATOR_STATE_DELETE:
      if (event == INITIATOR_EVT_PROCEDURE_ENABLE_COMPLETED) {
        sc = state_delete_on_procedure_enable_completed(initiator, data);
      }
      break;

    case INITIATOR_STATE_ERROR:
      break;

    default:
      break;
  }
  return sc;
}

static sl_status_t state_any_on_error(cs_initiator_t             *initiator,
                                      state_machine_event_data_t *data)
{
  app_timer_stop(&initiator->timer_handle);
  initiator_log_error(INSTANCE_PREFIX "Instance new state: ERROR" LOG_NL, initiator->conn_handle);
  initiator->initiator_state = INITIATOR_STATE_ERROR;
  on_error(initiator->conn_handle,
           data->evt_error.error_type,
           data->evt_error.sc);
  return SL_STATUS_OK;
}

static sl_status_t state_any_on_delete_instance(cs_initiator_t             *initiator,
                                                state_machine_event_data_t *data)
{
  (void)data;
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;

  sc = sl_bt_cs_procedure_enable(initiator->conn_handle,
                                 sl_bt_cs_procedure_state_disabled,
                                 initiator->config.config_id);

  if (sc == SL_STATUS_OK) {
    initiator_log_info(INSTANCE_PREFIX "Instance new state: DELETING" LOG_NL,
                       initiator->conn_handle);
    initiator->initiator_state = INITIATOR_STATE_DELETE;
  } else if (sc == SL_STATUS_BT_CTRL_COMMAND_DISALLOWED
             || sc == SL_STATUS_INVALID_HANDLE) {
    // Procedure is already stopped by stack
    app_timer_stop(&initiator->timer_handle);
    sc = initiator_finalize_cleanup(initiator);
    if (sc == SL_STATUS_OK) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: UNINITIALIZED" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_UNINITIALIZED;
      return SL_STATUS_OK;
    } else {
      initiator->initiator_state = INITIATOR_STATE_ERROR;
      data_out.evt_error.error_type = CS_ERROR_EVENT_INITIATOR_FAILED_TO_FINALIZE_CLEANUP;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
      return sc;
    }
  } else {
    // no other error code will be returned for now
    // if there will be one in the future it will generate an error state
    initiator_log_error(INSTANCE_PREFIX "failed to stop CS procedure! [sc: 0x%lx]" LOG_NL,
                        initiator->conn_handle,
                        (unsigned long)sc);
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_FAILED;
    data_out.evt_error.sc = sc;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               NULL);
  }

  return sc;
}

static sl_status_t state_uninitialized_on_init_started(cs_initiator_t             *initiator,
                                                       state_machine_event_data_t *data)
{
  (void)data;
  initiator_log_info(INSTANCE_PREFIX "Instance new state: INIT" LOG_NL,
                     initiator->conn_handle);
  initiator->initiator_state = INITIATOR_STATE_INIT;

  return SL_STATUS_OK;
}

static sl_status_t state_init_on_start_init_completed(cs_initiator_t             *initiator,
                                                      state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;

  if (data->evt_init_completed) {
    initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                       initiator->conn_handle);
    initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
    initiator->procedure_enable_retry_counter = 0;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_START_PROCEDURE,
                                               NULL);
  } else {
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_INIT_FAILED;
    data_out.evt_error.sc = SL_STATUS_FAIL;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
  }

  return sc;
}

static sl_status_t state_start_procedure_on_start_procedure(cs_initiator_t             *initiator,
                                                            state_machine_event_data_t *data)
{
  (void)data;
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;

  initiator_log_debug(INSTANCE_PREFIX "CS procedure - request to start." LOG_NL,
                      initiator->conn_handle);
  // Before enabling procedure, check the security state
  if (!initiator->cs_security_enabled) {
    // Security is not enabled, move to error state
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_INITIATOR_FAILED_TO_ENABLE_CS_SECURITY;
    data_out.evt_error.sc = sc;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
    return sc;
  }
  sc = sl_bt_cs_procedure_enable(initiator->conn_handle,
                                 sl_bt_cs_procedure_state_enabled,
                                 initiator->config.config_id);
  if (sc == SL_STATUS_OK) {
    // Start timer for procedure timeout
    // The timer is stopped when the procedure is completed/aborted
    // for both the initiator and the reflector or when there was an error
    sc = app_timer_start(&initiator->timer_handle,
                         CS_INITIATOR_PROCEDURE_TIMEOUT_MS,
                         procedure_timer_cb,
                         (void *)initiator,
                         false);
    if (sc == SL_STATUS_OK) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_PROCEDURE_ENABLE_COMPLETE" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_WAIT_PROCEDURE_ENABLE_COMPLETE;
      sc = SL_STATUS_OK;
    } else {
      initiator->initiator_state = INITIATOR_STATE_ERROR;
      data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_START_TIMER_FAILED;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
    }
  } else {
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_START_FAILED;
    data_out.evt_error.sc = sc;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
  }

  return sc;
}

static sl_status_t state_wait_procedure_enable_complete_on_enable_completed(cs_initiator_t             *initiator,
                                                                            state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;

  if (data->evt_procedure_enable_completed->status == SL_STATUS_OK) {
    uint32_t time_tick = sl_sleeptimer_get_tick_count();
    initiator->procedure_start_time_ms =
      sl_sleeptimer_tick_to_ms(time_tick);

    initiator->cs_procedure.cs_config.subevent_len
      = data->evt_procedure_enable_completed->subevent_len;
    initiator->cs_procedure.cs_config.subevent_interval =
      data->evt_procedure_enable_completed->subevent_interval;
    initiator_log_info(INSTANCE_PREFIX "-------------------------------------------" LOG_NL,
                       initiator->conn_handle);
    initiator_log_info(INSTANCE_PREFIX "CS procedure - started @ %lu ms (%lu tick) "
                                       "subevent_len: %lu, "
                                       "subevent_interval: %u]" LOG_NL,
                       initiator->conn_handle,
                       (unsigned long)initiator->procedure_start_time_ms,
                       (unsigned long)time_tick,
                       (unsigned long)initiator->cs_procedure.cs_config.subevent_len,
                       initiator->cs_procedure.cs_config.subevent_interval);

    initiator_log_info(INSTANCE_PREFIX "Instance new state: IN_PROCEDURE" LOG_NL,
                       initiator->conn_handle);
    initiator->initiator_state = INITIATOR_STATE_IN_PROCEDURE;
    sc = SL_STATUS_OK;
  } else {
    initiator_log_error(INSTANCE_PREFIX "CS procedure - start received error response! [status: 0x%x]" LOG_NL,
                        initiator->conn_handle,
                        data->evt_procedure_enable_completed->status);
    initiator->procedure_enable_retry_counter++;
    if (initiator->procedure_enable_retry_counter < CS_MAX_PROCEDURE_ENABLE_RETRY ) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_START_PROCEDURE,
                                                 NULL);
      return sc;
    }
    app_timer_stop(&initiator->timer_handle);
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_ENABLE_FAILED;
    data_out.evt_error.sc = SL_STATUS_FAIL;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
  }

  return sc;
}

static sl_status_t state_in_procedure_on_cs_result(cs_initiator_t             *initiator,
                                                   state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  cs_procedure_state_t procedure_state;

  procedure_state = extract_cs_result_data(initiator,
                                           data->evt_cs_result.initiator_part,
                                           data->evt_cs_result.cs_result_data);
  if (data->evt_cs_result.initiator_part == true) {
    initiator->initiator_data.procedure_counter = data->evt_cs_result.cs_result_data->procedure_counter;
    initiator_log_debug(INSTANCE_PREFIX "Initiator CS packet received. [proc_cnt=%u, "
                                        "proc_done=%u se_done=%u]" LOG_NL,
                        initiator->conn_handle,
                        data->evt_cs_result.cs_result_data->procedure_counter,
                        data->evt_cs_result.cs_result_data->procedure_done_status,
                        data->evt_cs_result.cs_result_data->subevent_done_status);
    switch (procedure_state) {
      case CS_PROCEDURE_STATE_IN_PROGRESS:
        initiator->initiator_state = INITIATOR_STATE_IN_PROCEDURE;
        sc = SL_STATUS_OK;
        break;
      case CS_PROCEDURE_STATE_ABORTED:
        initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_REFLECTOR_PROCEDURE_ABORTED" LOG_NL,
                           initiator->conn_handle);
        initiator->initiator_state = INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_ABORTED;
        sc = SL_STATUS_OK;
        break;
      case CS_PROCEDURE_STATE_COMPLETED:
        initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_REFLECTOR_PROCEDURE_COMPLETE" LOG_NL,
                           initiator->conn_handle);
        initiator->initiator_state = INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_COMPLETE;
        sc = SL_STATUS_OK;
        break;
      default:
        break;
    }
  } else {
    initiator->reflector_data.procedure_counter = data->evt_cs_result.cs_result_data->procedure_counter;
    initiator_log_debug(INSTANCE_PREFIX "Reflector CS packet received. [proc_cnt=%u, "
                                        "proc_done=%u se_done=%u]" LOG_NL,
                        initiator->conn_handle,
                        data->evt_cs_result.cs_result_data->procedure_counter,
                        data->evt_cs_result.cs_result_data->procedure_done_status,
                        data->evt_cs_result.cs_result_data->subevent_done_status);
    if (procedure_state == CS_PROCEDURE_STATE_IN_PROGRESS) {
      initiator->initiator_state = INITIATOR_STATE_IN_PROCEDURE;
      sc = SL_STATUS_OK;
    } else if (procedure_state == CS_PROCEDURE_STATE_ABORTED) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_INITIATOR_PROCEDURE_ABORTED" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_WAIT_INITIATOR_PROCEDURE_ABORTED;
      sc = SL_STATUS_OK;
    } else {
      // we get completed from reflector first - cannot happened - procedure disable
      initiator->initiator_state = initiator_stop_procedure_on_invalid_state(initiator);
      sc = SL_STATUS_OK;
      if (initiator->initiator_state == INITIATOR_STATE_ERROR) {
        data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA;
        data_out.evt_error.sc = sc;
        sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                   INITIATOR_EVT_ERROR,
                                                   &data_out);
      }
    }
  }

  return sc;
}

static sl_status_t state_wait_reflector_procedure_complete_on_cs_result(cs_initiator_t             *initiator,
                                                                        state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  cs_procedure_state_t procedure_state;

  if (data->evt_cs_result.initiator_part) {
    // we have already received procedure_completed from initiator
    // yet more cs_result_id event are coming - disable procedure
    initiator->initiator_state = initiator_stop_procedure_on_invalid_state(initiator);
    sc = SL_STATUS_OK;
    if (initiator->initiator_state == INITIATOR_STATE_ERROR) {
      data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
    }
    return sc;
  }

  procedure_state = extract_cs_result_data(initiator,
                                           data->evt_cs_result.initiator_part,
                                           data->evt_cs_result.cs_result_data);
  switch (procedure_state) {
    case CS_PROCEDURE_STATE_IN_PROGRESS:
      initiator->initiator_state = INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_COMPLETE;
      sc = SL_STATUS_OK;
      break;
    case CS_PROCEDURE_STATE_ABORTED:
      sc = app_timer_stop(&initiator->timer_handle);
      if (sc == SL_STATUS_OK) {
        initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                           initiator->conn_handle);
        initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
        sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                   INITIATOR_EVT_START_PROCEDURE,
                                                   NULL);
      } else {
        initiator->initiator_state = INITIATOR_STATE_ERROR;
        data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED;
        data_out.evt_error.sc = sc;
        sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                   INITIATOR_EVT_ERROR,
                                                   &data_out);
      }
      break;
    case CS_PROCEDURE_STATE_COMPLETED:
      if (initiator->initiator_data.procedure_counter == initiator->reflector_data.procedure_counter) {
        calculate_distance(initiator->conn_handle);
        sc = app_timer_stop(&initiator->timer_handle);
        if (sc == SL_STATUS_OK) {
          if (initiator->config.max_procedure_count != 0) {
            initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                               initiator->conn_handle);
            initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
            sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                       INITIATOR_EVT_START_PROCEDURE,
                                                       NULL);
            return sc;
          }
          initiator_log_info(INSTANCE_PREFIX "Instance new state: IN_PROCEDURE" LOG_NL,
                             initiator->conn_handle);
          initiator->initiator_state = INITIATOR_STATE_IN_PROCEDURE;
          sc = SL_STATUS_OK;
        } else {
          initiator->initiator_state = INITIATOR_STATE_ERROR;
          data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED;
          data_out.evt_error.sc = sc;
          sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                     INITIATOR_EVT_ERROR,
                                                     &data_out);
        }
      } else {
        initiator->initiator_state = INITIATOR_STATE_ERROR;
        data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_COUNTER_MISMATCH;
        data_out.evt_error.sc = sc;
        sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                   INITIATOR_EVT_ERROR,
                                                   &data_out);
      }
      break;
    default:
      break;
  }

  return sc;
}

static sl_status_t state_wait_reflector_procedure_aborted_on_cs_result(cs_initiator_t             *initiator,
                                                                       state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  cs_procedure_state_t procedure_state;

  if (data->evt_cs_result.initiator_part) {
    // we have already received procedure_aborted from initiator
    // yet more cs_result_id event are coming - disable procedure
    initiator->initiator_state = initiator_stop_procedure_on_invalid_state(initiator);
    sc = SL_STATUS_OK;
    if (initiator->initiator_state == INITIATOR_STATE_ERROR) {
      data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
    }
    return sc;
  }
  procedure_state = extract_cs_result_data(initiator,
                                           data->evt_cs_result.initiator_part,
                                           data->evt_cs_result.cs_result_data);
  if (procedure_state == CS_PROCEDURE_STATE_ABORTED) {
    sc = app_timer_stop(&initiator->timer_handle);
    if (sc == SL_STATUS_OK) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_START_PROCEDURE,
                                                 NULL);
      return sc;
    }
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED;
    data_out.evt_error.sc = sc;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
    return sc;
  }
  initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_REFLECTOR_PROCEDURE_ABORTED" LOG_NL,
                     initiator->conn_handle);
  initiator->initiator_state = INITIATOR_STATE_WAIT_REFLECTOR_PROCEDURE_ABORTED;
  sc = SL_STATUS_OK;

  return sc;
}

static sl_status_t state_wait_initiator_procedure_aborted_on_cs_result(cs_initiator_t             *initiator,
                                                                       state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  cs_procedure_state_t procedure_state;

  if (data->evt_cs_result.initiator_part == false) {
    // we have already received procedure_aborted from reflector
    // yet more gatt event are coming - disable procedure
    initiator->initiator_state = initiator_stop_procedure_on_invalid_state(initiator);
    sc = SL_STATUS_OK;
    if (initiator->initiator_state == INITIATOR_STATE_ERROR) {
      data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_UNEXPECTED_DATA;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
    }
    return sc;
  }

  procedure_state = extract_cs_result_data(initiator,
                                           data->evt_cs_result.initiator_part,
                                           data->evt_cs_result.cs_result_data);
  if (procedure_state == CS_PROCEDURE_STATE_ABORTED) {
    sc = app_timer_stop(&initiator->timer_handle);
    if (sc == SL_STATUS_OK) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_START_PROCEDURE,
                                                 NULL);
    } else {
      initiator->initiator_state = INITIATOR_STATE_ERROR;
      data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_TIMER_FAILED;
      data_out.evt_error.sc = sc;
      sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                                 INITIATOR_EVT_ERROR,
                                                 &data_out);
    }
  } else {
    initiator_log_info(INSTANCE_PREFIX "Instance new state: WAIT_INITIATOR_PROCEDURE_ABORTED" LOG_NL,
                       initiator->conn_handle);
    initiator->initiator_state = INITIATOR_STATE_WAIT_INITIATOR_PROCEDURE_ABORTED;
    sc = SL_STATUS_OK;
  }
  return sc;
}

static sl_status_t state_wait_procedure_disable_on_procedure_enable_completed(cs_initiator_t             *initiator,
                                                                              state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  if (data->evt_procedure_enable_completed->state) {
    return SL_STATUS_OK;
  }
  handle_procedure_enable_completed_event_disable(initiator);
  if (data->evt_procedure_enable_completed->status == SL_STATUS_OK) {
    app_timer_stop(&initiator->timer_handle);
    initiator_log_info(INSTANCE_PREFIX "Instance new state: START_PROCEDURE" LOG_NL,
                       initiator->conn_handle);
    initiator->initiator_state = INITIATOR_STATE_START_PROCEDURE;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_START_PROCEDURE,
                                               NULL);
    return sc;
  }
  initiator_log_error(INSTANCE_PREFIX "CS procedure - stop received error response! [status: 0x%x]" LOG_NL,
                      initiator->conn_handle,
                      data->evt_procedure_enable_completed->status);
  initiator->initiator_state = INITIATOR_STATE_ERROR;
  data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_FAILED;
  data_out.evt_error.sc = data->evt_procedure_enable_completed->status;
  sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                             INITIATOR_EVT_ERROR,
                                             &data_out);
  return sc;
}

static sl_status_t state_delete_on_procedure_enable_completed(cs_initiator_t             *initiator,
                                                              state_machine_event_data_t *data)
{
  sl_status_t sc = SL_STATUS_FAIL;
  state_machine_event_data_t data_out;
  if (data->evt_procedure_enable_completed->state) {
    return SL_STATUS_OK;
  }
  if (data->evt_procedure_enable_completed->status == SL_STATUS_OK) {
    // Procedure was disabled
    handle_procedure_enable_completed_event_disable(initiator);
    sc = initiator_finalize_cleanup(initiator);
    if (sc == SL_STATUS_OK) {
      initiator_log_info(INSTANCE_PREFIX "Instance new state: UNINITIALIZED" LOG_NL,
                         initiator->conn_handle);
      initiator->initiator_state = INITIATOR_STATE_UNINITIALIZED;
      return sc;
    }
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_INITIATOR_FAILED_TO_FINALIZE_CLEANUP;
    data_out.evt_error.sc = sc;
    sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                               INITIATOR_EVT_ERROR,
                                               &data_out);
    return sc;
  }
  // Only happens if the remote device rejects the procedure termination - error state
  initiator_log_error(INSTANCE_PREFIX "CS procedure - stop received error response! [status: 0x%x]" LOG_NL,
                      initiator->conn_handle,
                      data->evt_procedure_enable_completed->status);
  initiator->initiator_state = INITIATOR_STATE_ERROR;
  data_out.evt_error.error_type = CS_ERROR_EVENT_CS_PROCEDURE_STOP_FAILED;
  data_out.evt_error.sc = data->evt_procedure_enable_completed->status;
  sc = initiator_state_machine_event_handler(initiator->conn_handle,
                                             INITIATOR_EVT_ERROR,
                                             &data_out);
  return sc;
}

/******************************************************************************
 * State machine dedicated for the discovery process:
 * discovering services on reflector side and checking the characteristics.
 * Once everything is compatible, triggers the configuration of this initiator
 * device.
 *****************************************************************************/
static bool gatt_procedure_complete_event(const uint8_t conn_handle,
                                          sl_bt_msg_t    *evt)
{
  bool handled = false;
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL, conn_handle);
    on_error(conn_handle,
             CS_ERROR_EVENT_CS_PROCEDURE_COMPLETE_FAILED,
             SL_STATUS_NULL_POINTER);
    return handled;
  }
  state_machine_event_data_t evt_data;
  uint8_t evt_conn_handle = evt->data.evt_gatt_procedure_completed.connection;
  if (evt_conn_handle != initiator->conn_handle) {
    return handled;
  }
  uint16_t procedure_result =  evt->data.evt_gatt_procedure_completed.result;

  sl_status_t sc;

  if (procedure_result != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "GATT procedure failed" LOG_NL,
                        initiator->conn_handle);
    if (initiator->discovery_state.action != CS_TRANSPORT_STATE_IDLE) {
      on_error(initiator->conn_handle,
               CS_ERROR_EVENT_GATT_PROCEDURE_FAILED,
               procedure_result);
    }

    return handled;
  }

  switch (initiator->discovery_state.action) {
    case CS_TRANSPORT_START_DISCOVERING_RAS_CHARACTERISTICS:
      handled = true;
      initiator_log_info(INSTANCE_PREFIX "RAS - service discovered." LOG_NL,
                         initiator->conn_handle);
      // Service discovery successful, start characteristic discovery
      sc = sl_bt_gatt_discover_characteristics(initiator->conn_handle,
                                               initiator->ras_service_handle);
      if (sc != SL_STATUS_OK) {
        initiator_log_error(INSTANCE_PREFIX "RAS - starting characteristic discovery failed!" LOG_NL,
                            initiator->conn_handle);
        on_error(initiator->conn_handle,
                 CS_ERROR_EVENT_START_CHARACTERISTIC_DISCOVERY_FAILED,
                 sc);
        break;
      }
      // restart error timer after state transition
      start_error_timer(initiator->conn_handle);
      initiator->discovery_state.action = CS_TRANSPORT_WAIT_FOR_RAS_CHARACTERISTICS_DISCOVERY_COMPLETE;
      initiator_log_info(INSTANCE_PREFIX "RAS - discover characteristics started" LOG_NL,
                         initiator->conn_handle);
      break;

    case CS_TRANSPORT_WAIT_FOR_RAS_CHARACTERISTICS_DISCOVERY_COMPLETE:
      handled = true;
      if (!initiator->discovery_state.characteristic_found.ras_all) {
        break;
      }
      // Subscribe for notifications on the RAS Subevent Ranging Data
      sc = sl_bt_gatt_set_characteristic_notification(initiator->conn_handle,
                                                      initiator->characteristic_handle.ras_subevent_ranging_data,
                                                      sl_bt_gatt_notification);
      if (sc != SL_STATUS_OK) {
        initiator_log_error(INSTANCE_PREFIX "RAS - subscribe to \'Subevent Ranging Data\' failed!" LOG_NL,
                            initiator->conn_handle);
        on_error(initiator->conn_handle,
                 CS_ERROR_EVENT_SEND_NOTIFICATION_FAILED,
                 sc);
        break;
      }
      // restart error timer after state transition
      start_error_timer(initiator->conn_handle);
      initiator->discovery_state.action = CS_TRANSPORT_SUBSCRIBE_RAS_SUBEVENT_RANGING_DATA;
      break;

    case CS_TRANSPORT_SUBSCRIBE_RAS_SUBEVENT_RANGING_DATA:
      handled = true;
      initiator_log_info(INSTANCE_PREFIX "RAS - subscribed to \'Subevent Ranging Data\' notifications" LOG_NL,
                         initiator->conn_handle);
      // Subscribe for indications on the RAS Control Point
      sc = sl_bt_gatt_set_characteristic_notification(initiator->conn_handle,
                                                      initiator->characteristic_handle.ras_control_point,
                                                      sl_bt_gatt_indication);
      if (sc != SL_STATUS_OK) {
        initiator_log_error(INSTANCE_PREFIX "RAS - subscribe to \'Control Point\' failed!" LOG_NL,
                            initiator->conn_handle);
        on_error(initiator->conn_handle,
                 CS_ERROR_EVENT_SEND_INDICATION_FAILED,
                 sc);
        break;
      }
      // restart error timer after state transition
      start_error_timer(initiator->conn_handle);
      initiator->discovery_state.action = CS_TRANSPORT_SUBSCRIBE_RAS_CONTROL_POINT;
      break;

    case CS_TRANSPORT_SUBSCRIBE_RAS_CONTROL_POINT:
      handled = true;
      initiator_log_info(INSTANCE_PREFIX "RAS - subscribed to \'Control Point\' notifications." LOG_NL,
                         initiator->conn_handle);
      if (initiator->error_timer_started) {
        stop_error_timer(initiator->conn_handle);
      }
      evt_data.evt_init_completed = true;
      (void)initiator_state_machine_event_handler(initiator->conn_handle,
                                                  INITIATOR_EVT_INIT_COMPLETED,
                                                  &evt_data);
      break;

    case CS_TRANSPORT_STATE_IDLE:
      break;

    default:
      break;
  }
  return handled;
}

/******************************************************************************
 * Check if the detected characteristic matches the UUIDs that
 * is required for the CS functionality
 *****************************************************************************/
static bool check_characteristic_uuid(const uint8_t conn_handle,
                                      sl_bt_msg_t *evt)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  bool handled = false;
  if (evt->data.evt_gatt_characteristic.uuid.len == UUID_LEN) {
    if (memcmp(ras_control_point_characteristic_uuid,
               evt->data.evt_gatt_characteristic.uuid.data,
               UUID_LEN) == 0) {
      initiator->characteristic_handle.ras_control_point =
        evt->data.evt_gatt_characteristic.characteristic;
      initiator->discovery_state.characteristic_found.ras_control_point = true;
      initiator_log_info(INSTANCE_PREFIX "RAS discovered Control Point characteristic" LOG_NL,
                         initiator->conn_handle);
      start_error_timer(initiator->conn_handle);
      handled = true;
    }

    if (memcmp(ras_procedure_enable_data_characteristic_uuid,
               evt->data.evt_gatt_characteristic.uuid.data,
               UUID_LEN) == 0) {
      initiator->characteristic_handle.ras_procedure_enable_data =
        evt->data.evt_gatt_characteristic.characteristic;
      initiator->discovery_state.characteristic_found.ras_procedure_enable_data = true;
      initiator_log_info(INSTANCE_PREFIX "RAS discovered Procedure Enable Data characteristic" LOG_NL,
                         initiator->conn_handle);
      start_error_timer(initiator->conn_handle);
      handled = true;
    }

    if (memcmp(ras_se_ranging_data_characteristic_uuid,
               evt->data.evt_gatt_characteristic.uuid.data,
               UUID_LEN) == 0) {
      initiator->characteristic_handle.ras_subevent_ranging_data =
        evt->data.evt_gatt_characteristic.characteristic;
      initiator->discovery_state.characteristic_found.ras_subevent_ranging_data = true;
      initiator_log_info(INSTANCE_PREFIX "RAS discovered Subevent Ranging Data characteristic" LOG_NL,
                         initiator->conn_handle);
      start_error_timer(initiator->conn_handle);
      handled = true;
    }

    if (initiator->discovery_state.characteristic_found.ras_control_point
        && initiator->discovery_state.characteristic_found.ras_procedure_enable_data
        && initiator->discovery_state.characteristic_found.ras_subevent_ranging_data) {
      initiator->discovery_state.characteristic_found.ras_all = true;
      initiator_log_info(INSTANCE_PREFIX "RAS all characteristics have been discovered successfully" LOG_NL,
                         initiator->conn_handle);
    }
  }
  return handled;
}

/******************************************************************************
 * Extract cs results (step data, subevent data) and hand over to RTL Lib
 * instance to calculate distance out of it.
 *
 * @return true if procedure restart required, false otherwise
 *****************************************************************************/
static cs_procedure_state_t extract_cs_result_data(cs_initiator_t *initiator,
                                                   bool initiator_part,
                                                   sl_bt_evt_cs_result_t *evt)
{
  cs_procedure_state_t procedure_state = CS_PROCEDURE_STATE_IN_PROGRESS;
  if (initiator_part) {
    initiator_log_info(INSTANCE_PREFIX "extract - initiator data" LOG_NL,
                       initiator->conn_handle);
    procedure_state = extract_device_cs_data(initiator->conn_handle,
                                             &initiator->initiator_data,
                                             initiator->cs_procedure.initiator_subevent_data,
                                             &initiator->cs_procedure.initiator_subevent_data_count,
                                             evt);
  } else {
    initiator_log_info(INSTANCE_PREFIX "extract - reflector data" LOG_NL,
                       initiator->conn_handle);
    procedure_state = extract_device_cs_data(initiator->conn_handle,
                                             &initiator->reflector_data,
                                             initiator->cs_procedure.reflector_subevent_data,
                                             &initiator->cs_procedure.reflector_subevent_data_count,
                                             evt);
  }

  initiator_log_info("----" LOG_NL);
  return procedure_state;
}

static cs_procedure_state_t extract_device_cs_data(const uint8_t conn_handle,
                                                   cs_data_t *device_data,
                                                   sl_rtl_cs_subevent_data *rtl_subevent_data,
                                                   uint8_t *rtl_subevent_data_count,
                                                   sl_bt_evt_cs_result_t *evt)
{
  cs_procedure_state_t procedure_state = CS_PROCEDURE_STATE_IN_PROGRESS;
  uint16_t subevent_idx = 0u;
  uint16_t step_idx = 0u;
  uint32_t subevent_time_tick = sl_sleeptimer_get_tick_count();
  uint32_t subevent_time_ms;
  size_t step_data_len = 0u;
  state_machine_event_data_t data_out;

  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);

  subevent_idx = device_data->subevent_index;

  if (device_data->subevent_data[subevent_idx].num_steps == 0u) {
    // assign step_data array address to the sub events' pointer element
    device_data->subevent_data[subevent_idx].step_data =
      &device_data->step_data[step_idx];

    // assign devices step_data array to the procedure data's sub event pointer element too
    rtl_subevent_data->step_data =
      device_data->subevent_data[subevent_idx].step_data;
  }

  subevent_time_ms = sl_sleeptimer_tick_to_ms(subevent_time_tick);
  initiator_log_info(INSTANCE_PREFIX "extract @ %lu ms (%lu ticks)" LOG_NL,
                     conn_handle,
                     (unsigned long)subevent_time_ms,
                     (unsigned long)subevent_time_tick);

  // set timestamp, freq. compensation data and reference power level
  device_data->subevent_data[subevent_idx].subevent_timestamp_ms =
    subevent_time_ms;
  device_data->subevent_data[subevent_idx].frequency_compensation =
    evt->frequency_compensation;
  device_data->subevent_data[subevent_idx].reference_power_level
    = evt->reference_power_level;

  // check step data size
  step_idx = device_data->step_index;
  step_data_len = step_idx + evt->data.len;

  if (step_data_len > CS_INITIATOR_MAX_STEP_DATA_LEN) {
    initiator_log_error(INSTANCE_PREFIX "extract - incoming step data exceeds step data "
                                        " buffer size: %u/%u!" LOG_NL,
                        conn_handle,
                        (unsigned int)step_data_len,
                        (unsigned int)CS_INITIATOR_MAX_STEP_DATA_LEN);
    reset_subevent_data(conn_handle);
    initiator->initiator_state = INITIATOR_STATE_ERROR;
    data_out.evt_error.error_type = CS_ERROR_EVENT_STEP_DATA_EXCEEDS_MAX;
    (void)initiator_state_machine_event_handler(conn_handle,
                                                INITIATOR_EVT_ERROR,
                                                &data_out);
  } else {
    // copy step data from cs_results event into the device data step_data buffer
    memcpy(&device_data->step_data[step_idx], evt->data.data, evt->data.len);
    initiator_log_debug(INSTANCE_PREFIX "extract - update step_index %u -> %u" LOG_NL,
                        conn_handle,
                        (unsigned int)step_idx,
                        (unsigned int)step_data_len);
    // increment step index
    step_idx += evt->data.len;
    // take back the step index
    device_data->step_index = step_idx;

    // update the number of step data bytes for logger
    device_data->subevent_data[subevent_idx].step_data_count = device_data->step_index;
    initiator_log_debug(INSTANCE_PREFIX "extract - updated num_steps %u -> %u [step_data_count: %u]" LOG_NL,
                        conn_handle,
                        device_data->subevent_data[subevent_idx].num_steps,
                        (device_data->subevent_data[subevent_idx].num_steps + evt->num_steps),
                        device_data->subevent_data[subevent_idx].step_data_count);

    // take over number of steps from cs_results event
    device_data->subevent_data[subevent_idx].num_steps += evt->num_steps;
  }

  // --------------------------------
  // Subevent state check
  if (evt->subevent_done_status == sl_bt_cs_done_status_complete) {
    // subevent arrived - increment data counter
    (*rtl_subevent_data_count)++;

    if (device_data->subevent_index < CS_INITIATOR_MAX_SUBEVENT_PER_PROC) {
      // update subevent index for the device
      device_data->subevent_index++;

      initiator_log_debug(INSTANCE_PREFIX "extract - procedure %u sub events %u/%u "
                                          "extracted [rtl_subevent_data_count=%u]" LOG_NL,
                          conn_handle,
                          evt->procedure_counter,
                          device_data->subevent_index,
                          CS_INITIATOR_MAX_SUBEVENT_PER_PROC,
                          *rtl_subevent_data_count);
    } else {
      reset_subevent_data(conn_handle);
      initiator_log_error(INSTANCE_PREFIX "extract - sub event data buffer is full! "
                                          "Data loss inevitable!" LOG_NL,
                          conn_handle);
      initiator->initiator_state = INITIATOR_STATE_ERROR;
      data_out.evt_error.error_type = CS_ERROR_EVENT_SUBEVENT_INDEX_EXCEEDS_MAX;
      (void)initiator_state_machine_event_handler(conn_handle,
                                                  INITIATOR_EVT_ERROR,
                                                  &data_out);
    }
  } else if (evt->subevent_done_status == sl_bt_cs_done_status_partial_results_continue) {
    initiator_log_debug(INSTANCE_PREFIX "extract - sub event partially arrived." LOG_NL,
                        conn_handle);
  } else if (evt->subevent_done_status == sl_bt_cs_done_status_aborted) {
    initiator_log_warning(INSTANCE_PREFIX "extract - procedure %u sub event aborted!"
                                          "[reason: 0x%X]" LOG_NL,
                          conn_handle,
                          evt->procedure_counter,
                          evt->abort_reason);
    reset_subevent_data(conn_handle);
  }
  // --------------------------------
  // Procedure state check
  if (evt->procedure_done_status == sl_bt_cs_done_status_complete) {
    initiator_log_info(INSTANCE_PREFIX "extract - device ready." LOG_NL,
                       conn_handle);

    // reset indexes in order to prepare to the new procedure
    device_data->subevent_index = 0u;
    device_data->step_index = 0u;
    device_data->ready = true;
    procedure_state = CS_PROCEDURE_STATE_COMPLETED;
  } else if (evt->procedure_done_status == sl_bt_cs_done_status_partial_results_continue) {
    initiator_log_debug(INSTANCE_PREFIX "extract - procedure %u partially done." LOG_NL,
                        conn_handle,
                        evt->procedure_counter);
    procedure_state = CS_PROCEDURE_STATE_IN_PROGRESS;
  } else if (evt->procedure_done_status == sl_bt_cs_done_status_aborted) {
    initiator_log_warning(INSTANCE_PREFIX "extract - procedure %u aborted! "
                                          "[reason: 0x%X]" LOG_NL,
                          conn_handle,
                          evt->procedure_counter,
                          evt->abort_reason);
    reset_subevent_data(conn_handle);
    procedure_state = CS_PROCEDURE_STATE_ABORTED;
  }
  return procedure_state;
}

/******************************************************************************
 * Show error messages based on RTL API call error codes.
 *
 * @param[in] conn_handle connection handle if the selected initiator instance.
 * @param[in] err_code RTL API error code.
 *****************************************************************************/
static void show_rtl_api_call_result(const uint8_t conn_handle,
                                     enum sl_rtl_error_code err_code)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    return;
  }

  switch (err_code) {
    case SL_RTL_ERROR_SUCCESS:
      initiator_log_debug(INSTANCE_PREFIX "RTL - no error." LOG_NL, initiator->conn_handle);
      break;

    case SL_RTL_ERROR_ARGUMENT:
      initiator_log_error(INSTANCE_PREFIX "RTL - invalid argument! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_OUT_OF_MEMORY:
      initiator_log_error(INSTANCE_PREFIX "RTL - memory allocation error! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_ESTIMATION_IN_PROGRESS:
      initiator_log_info(INSTANCE_PREFIX "RTL - estimation not yet finished! [E:%u]" LOG_NL,
                         initiator->conn_handle,
                         err_code);
      break;

    case SL_RTL_ERROR_NUMBER_OF_SNAPHOTS_DO_NOT_MATCH:
      initiator_log_error(INSTANCE_PREFIX "RTL - initialized and calculated "
                                          "snapshots do not match! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_ESTIMATOR_NOT_CREATED:
      initiator_log_error(INSTANCE_PREFIX "RTL - estimator not created! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_ESTIMATOR_ALREADY_CREATED:
      initiator_log_error(INSTANCE_PREFIX "RTL - estimator already created! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_NOT_INITIALIZED:
      initiator_log_error(INSTANCE_PREFIX "RTL - library item not initialized! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_INTERNAL:
      initiator_log_error(INSTANCE_PREFIX "RTL - internal error! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_IQ_SAMPLE_QA:
      initiator_log_error(INSTANCE_PREFIX "RTL - IQ sample quality analysis failed! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_FEATURE_NOT_SUPPORTED:
      initiator_log_error(INSTANCE_PREFIX "RTL - feature not supported! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    case SL_RTL_ERROR_INCORRECT_MEASUREMENT:
      initiator_log_error(INSTANCE_PREFIX "RTL - incorrect measurement! Error of the last"
                                          " measurement was too large! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;

    default:
      initiator_log_error(INSTANCE_PREFIX "RTL - unknown error! [E:%u]" LOG_NL,
                          initiator->conn_handle,
                          err_code);
      break;
  }
}

/******************************************************************************
 * Calculate distance between initiator and reflector using RTL library.
 *
 * @param[in] initiator Initiator instance.
 *****************************************************************************/
static void calculate_distance(const uint8_t conn_handle)
{
  cs_result_t result;
  cs_intermediate_result_t intermediate_result;
  enum sl_rtl_error_code rtl_err = SL_RTL_ERROR_NOT_INITIALIZED;

  bool estimation_valid = false;

  memset(&result, 0u, sizeof(result));
  result.connection = conn_handle;

  memset(&intermediate_result, 0u, sizeof(intermediate_result));
  intermediate_result.connection = conn_handle;

  sl_rtl_cs_estimator_param param;
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);

  initiator_log_info(INSTANCE_PREFIX "RTL - process CS results" LOG_NL,
                     initiator->conn_handle);
  if (initiator->rtl_handle == NULL) {
    initiator_log_error(INSTANCE_PREFIX "RTL - RTL handle is NULL!" LOG_NL,
                        initiator->conn_handle);
    reset_subevent_data(initiator->conn_handle);
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_RTL_ERROR,
             SL_STATUS_NULL_POINTER);
    return;
  }
  // procedure count is always 1.
  rtl_err = sl_rtl_cs_process(&initiator->rtl_handle,
                              1,
                              &initiator->cs_procedure);
  show_rtl_api_call_result(initiator->conn_handle, rtl_err);

  if (rtl_err != SL_RTL_ERROR_SUCCESS && rtl_err != SL_RTL_ERROR_ESTIMATION_IN_PROGRESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to process CS data; error='%u'" LOG_NL,
                        initiator->conn_handle,
                        rtl_err);
    reset_subevent_data(initiator->conn_handle);
    //on_error - app will decide what to do with RTL errors e.g start counter
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_RTL_PROCESS_ERROR,
             SL_STATUS_FAIL);
    return;
  }

  initiator_log_info(INSTANCE_PREFIX "RTL - get distance" LOG_NL,
                     initiator->conn_handle);
  rtl_err = sl_rtl_cs_get_distance_estimate(&initiator->rtl_handle,
                                            SL_RTL_CS_DISTANCE_ESTIMATE_TYPE_FILTERED,
                                            &result.distance);
  show_rtl_api_call_result(initiator->conn_handle, rtl_err);
  if (rtl_err == SL_RTL_ERROR_SUCCESS) {
    estimation_valid = true;
  } else if (rtl_err == SL_RTL_ERROR_ESTIMATION_IN_PROGRESS) {
    initiator_log_info(INSTANCE_PREFIX "RTL - estimation in progress" LOG_NL,
                       initiator->conn_handle);
    rtl_err =
      sl_rtl_cs_get_distance_estimate_extended_info(&initiator->rtl_handle,
                                                    SL_RTL_CS_DISTANCE_ESTIMATE_EXTENDED_INFO_TYPE_PROGRESS_PERCENTAGE,
                                                    &intermediate_result.progress_percentage);
    show_rtl_api_call_result(initiator->conn_handle, rtl_err);
    if (rtl_err == SL_RTL_ERROR_SUCCESS) {
      initiator_log_info("#%u: RTL - got estimation state" LOG_NL, initiator->conn_handle);
      if (initiator->intermediate_result_cb != NULL) {
        // Call intermediate result callback in case of in process call
        initiator->intermediate_result_cb(&intermediate_result, NULL);
      }
    } else {
      initiator_log_error("#%u: RTL - failed to get estimation extended info! [E: %u]" LOG_NL,
                          initiator->conn_handle,
                          rtl_err);
    }
    reset_subevent_data(initiator->conn_handle);
    return; // Intermediate results shall not be displayed or reported!
  } else {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to get distance data! [E: %u]" LOG_NL,
                        initiator->conn_handle,
                        rtl_err);
  }

  initiator_log_info(INSTANCE_PREFIX "RTL - get distance likeliness" LOG_NL,
                     initiator->conn_handle);
  rtl_err = sl_rtl_cs_get_distance_estimate_confidence(&initiator->rtl_handle,
                                                       SL_RTL_CS_DISTANCE_ESTIMATE_CONFIDENCE_TYPE_LIKELINESS,
                                                       &result.likeliness);
  show_rtl_api_call_result(initiator->conn_handle, rtl_err);
  if (rtl_err == SL_RTL_ERROR_SUCCESS) {
    initiator_log_info(INSTANCE_PREFIX "RTL - estimation valid." LOG_NL,
                       initiator->conn_handle);
    estimation_valid = true;
  } else {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to calculate distance likeliness! [E: %u]" LOG_NL,
                        initiator->conn_handle,
                        rtl_err);
  }

  // Set reference TX power for RSSI calculation
  param.type = SL_RTL_REF_TX_POWER;
  param.value.ref_tx_power = initiator->config.rssi_ref_tx_power;
  rtl_err = sl_rtl_cs_set_estimator_param(&initiator->rtl_handle, &param);
  if (rtl_err != SL_RTL_ERROR_SUCCESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to set RSSI reference TX power! %u" LOG_NL,
                        initiator->conn_handle,
                        rtl_err);
  }

  // Calculate distance from RSSI
  rtl_err = sl_rtl_cs_get_distance_estimate(&initiator->rtl_handle,
                                            SL_RTL_CS_DISTANCE_ESTIMATE_TYPE_RSSI,
                                            &result.rssi_distance);
  show_rtl_api_call_result(initiator->conn_handle, rtl_err);
  if (rtl_err == SL_RTL_ERROR_SUCCESS) {
    estimation_valid = true;
    param.type = SL_RTL_LAST_KNOWN_DISTANCE;
    param.value.last_known_distance = result.distance;
    rtl_err = sl_rtl_cs_set_estimator_param(&initiator->rtl_handle, &param);
    show_rtl_api_call_result(initiator->conn_handle, rtl_err);
  } else {
    initiator_log_error(INSTANCE_PREFIX "RTL - could not get RSSI distance data!" LOG_NL,
                        initiator->conn_handle);
  }

  if (estimation_valid && initiator->result_cb != NULL) {
    // Call result callback in case of successful process call
    initiator->result_cb(&result, NULL);
  }
  reset_subevent_data(initiator->conn_handle);
}

/******************************************************************************
 * Handle procedure_enable_completed event when procedure was
 * disabled successfully.
 * @param[in] initiator initiator instance
 *****************************************************************************/
static void handle_procedure_enable_completed_event_disable(cs_initiator_t *initiator)
{
  uint32_t time_tick = sl_sleeptimer_get_tick_count();
  initiator->procedure_stop_time_ms =
    sl_sleeptimer_tick_to_ms(time_tick);
  initiator_log_info(INSTANCE_PREFIX "CS procedure - stopped @ %lu ms "
                                     "(%lu tick)" LOG_NL,
                     initiator->conn_handle,
                     (unsigned long)initiator->procedure_stop_time_ms,
                     (unsigned long)time_tick);
}

/******************************************************************************
 * Get number of tones in channel map
 *
 * @param ch_map is the channel map data
 * @param ch_map_len is the channel map data length
 *****************************************************************************/
static uint32_t get_num_tones_from_channel_map(const uint8_t  *ch_map,
                                               const uint32_t ch_map_len)
{
  uint8_t current_ch_map;
  uint32_t num_cs_channels = 0;

  if (ch_map == NULL) {
    initiator_log_error("null reference to channel map! Can not get number of tones!" LOG_NL);
    return num_cs_channels;
  } else {
    for (uint32_t ch_map_index = 0; ch_map_index < ch_map_len; ch_map_index++) {
      current_ch_map = ch_map[ch_map_index];
      for (uint8_t current_bit_index = 0;
           current_bit_index < sizeof(uint8_t) * CHAR_BIT;
           current_bit_index++) {
        if (current_ch_map & (1 << current_bit_index)) {
          num_cs_channels++;
        }
      }
    }
  }
  return num_cs_channels;
}

/******************************************************************************
 * Call user error callback on error
 *
 * @param cs_initiator_t instance reference
 * @param cs_error_event_t initiator error event
 * @param sl_status_t status code
 *****************************************************************************/
static void on_error(const uint8_t conn_handle,
                     cs_error_event_t evt,
                     sl_status_t      sc)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    return;
  }

  cs_error_cb_t cb = initiator->error_cb;
  initiator_log_error(INSTANCE_PREFIX "Error occurred (sc: 0x%lx)" LOG_NL,
                      conn_handle,
                      (unsigned long)sc);
  if (cb != NULL) {
    cb(conn_handle, evt, sc);
  }
}

/******************************************************************************
 * Reset subevent data and synchronization for an initiator instance.
 *
 * @param cs_initiator_t instance reference
 *****************************************************************************/
static void reset_subevent_data(const uint8_t conn_handle)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    return;
  }

  initiator->initiator_data.step_index = 0u;
  initiator->reflector_data.step_index = 0u;

  initiator->initiator_data.subevent_index = 0u;
  initiator->reflector_data.subevent_index = 0u;

  // Assign subevent data pointer to cs_procedure subevent data pointer
  initiator->cs_procedure.initiator_subevent_data =
    initiator->initiator_data.subevent_data;
  initiator->cs_procedure.reflector_subevent_data =
    initiator->reflector_data.subevent_data;

  initiator->cs_procedure.initiator_subevent_data_count = 0u;
  initiator->cs_procedure.reflector_subevent_data_count = 0u;

  initiator->initiator_data.ready = false;
  initiator->reflector_data.ready = false;

  for (uint8_t i = 0; i < CS_INITIATOR_MAX_SUBEVENT_PER_PROC; i++) {
    initiator->initiator_data.subevent_data[i].num_steps = 0;
    // Assign step data pointer to cs_procedure step data pointer
    initiator->initiator_data.subevent_data[i].step_data =
      initiator->initiator_data.step_data;
    // Do it for the reflector side as well
    initiator->reflector_data.subevent_data[i].num_steps = 0;
    initiator->reflector_data.subevent_data[i].step_data =
      initiator->reflector_data.step_data;
  }

  initiator_log_info(INSTANCE_PREFIX "subevent data reset executed." LOG_NL,
                     initiator->conn_handle);
}

/******************************************************************************
 * RTL Lib init function
 *
 * @param[in] conn_handle connection handle
 * @param[in] handle RTL Lib handle reference
 * @param[in] config RTL configuration structure reference
 * @param[in] cs_mode CS measurement mode
 *
 * @return with RTL Lib specified error code
 *****************************************************************************/
static enum sl_rtl_error_code rtl_library_init(const uint8_t conn_handle,
                                               sl_rtl_cs_libitem *handle,
                                               rtl_config_t      *config,
                                               const uint8_t     cs_mode)
{
  enum sl_rtl_error_code rtl_err;

  if (handle != NULL && *handle != NULL) {
    rtl_err = sl_rtl_cs_deinit(handle);
    if (rtl_err != SL_RTL_ERROR_SUCCESS) {
      initiator_log_error(INSTANCE_PREFIX "RTL - failed to deinit lib! "
                                          "[err: 0x%02x]" LOG_NL,
                          conn_handle,
                          rtl_err);
      return rtl_err;
    }
    *handle = NULL;
  }

  rtl_err = sl_rtl_cs_init(handle);
  if (rtl_err != SL_RTL_ERROR_SUCCESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to init lib! "
                                        "[err: 0x%02x]" LOG_NL,
                        conn_handle,
                        rtl_err);
    return rtl_err;
  }

  if (config->rtl_logging_enabled) {
    sl_rtl_cs_log_params log_params;
    uint32_t time_tick = sl_sleeptimer_get_tick_count();
    log_params.timestamp.tv_sec = 0;
    log_params.timestamp.tv_nsec = 0;
    log_params.timestamp_ms = sl_sleeptimer_tick_to_ms(time_tick);
    rtl_err = sl_rtl_cs_log_enable(handle, &log_params);
    if (rtl_err != SL_RTL_ERROR_SUCCESS) {
      initiator_log_error(INSTANCE_PREFIX "RTL - failed to enable log! "
                                          "[err: 0x%02x]" LOG_NL,
                          conn_handle,
                          rtl_err);
      return rtl_err;
    }
  }

  rtl_err = sl_rtl_cs_set_algo_mode(handle, config->algo_mode);
  if (rtl_err != SL_RTL_ERROR_SUCCESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - set algo mode failed! "
                                        "[err: 0x%02x]" LOG_NL,
                        conn_handle,
                        rtl_err);
    return rtl_err;
  }

  switch (cs_mode) {
    case sl_bt_cs_mode_rtt:
      initiator_log_info(INSTANCE_PREFIX "RTL -  set CS mode: RTT" LOG_NL, conn_handle);
      rtl_err = sl_rtl_cs_set_cs_mode(handle, SL_RTL_CS_MODE_RTT);
      if (rtl_err != SL_RTL_ERROR_SUCCESS) {
        initiator_log_error(INSTANCE_PREFIX "RTL - failed to set CS mode! "
                                            "[err: 0x%02x]" LOG_NL,
                            conn_handle,
                            rtl_err);
        return rtl_err;
      }
      break;
    case sl_bt_cs_mode_pbr:
    {
      config->cs_parameters.max_number_of_frequencies =
        (uint32_t)DEFAULT_NUM_TONES;
      config->cs_parameters.delta_f =
        (uint32_t)DEFAULT_FREQUENCY_DELTA;

      initiator_log_info(INSTANCE_PREFIX "RTL - set CS mode: PBR" LOG_NL, conn_handle);
      rtl_err = sl_rtl_cs_set_cs_mode(handle, SL_RTL_CS_MODE_PBR);
      if (rtl_err != SL_RTL_ERROR_SUCCESS) {
        initiator_log_error(INSTANCE_PREFIX "RTL - failed to set CS mode! "
                                            "[err: 0x%02x]" LOG_NL,
                            conn_handle,
                            rtl_err);
        return rtl_err;
      }
      initiator_log_debug(INSTANCE_PREFIX "RTL - CS mode set. Set CS parameters "
                                          "[max_number_of_frequencies=%lu, delta_f=%lu]" LOG_NL,
                          conn_handle,
                          (unsigned long)config->cs_parameters.max_number_of_frequencies,
                          (unsigned long)config->cs_parameters.delta_f);
      rtl_err = sl_rtl_cs_set_cs_params(handle, &config->cs_parameters);
      if (rtl_err != SL_RTL_ERROR_SUCCESS) {
        initiator_log_error(INSTANCE_PREFIX "RTL - failed to set CS parameters! "
                                            "[err: 0x%02x]" LOG_NL,
                            conn_handle,
                            rtl_err);
        return rtl_err;
      }
    }
    break;
    default:
    {
      config->cs_parameters.max_number_of_frequencies =
        (uint32_t)DEFAULT_NUM_TONES;
      config->cs_parameters.delta_f = (uint32_t)DEFAULT_FREQUENCY_DELTA;

      initiator_log_info(INSTANCE_PREFIX "RTL - set CS mode to default [PBR]" LOG_NL,
                         conn_handle);
      rtl_err = sl_rtl_cs_set_cs_mode(handle, SL_RTL_CS_MODE_PBR);
      if (rtl_err != SL_RTL_ERROR_SUCCESS) {
        initiator_log_error(INSTANCE_PREFIX "RTL - failed to set CS mode! "
                                            "[err: 0x%02x]" LOG_NL,
                            conn_handle,
                            rtl_err);
        return rtl_err;
      }
      initiator_log_debug(INSTANCE_PREFIX "RTL - CS mode set. Set CS parameters "
                                          "[max_number_of_frequencies=%lu, delta_f=%lu]" LOG_NL,
                          conn_handle,
                          (unsigned long)config->cs_parameters.max_number_of_frequencies,
                          (unsigned long)config->cs_parameters.delta_f);
      rtl_err = sl_rtl_cs_set_cs_params(handle, &config->cs_parameters);
      if (rtl_err != SL_RTL_ERROR_SUCCESS) {
        initiator_log_error(INSTANCE_PREFIX "RTL - failed to set CS parameters! "
                                            "[err: 0x%02x]" LOG_NL,
                            conn_handle,
                            rtl_err);
        return rtl_err;
      }
    }
    break;
  }

  if (rtl_err == SL_RTL_ERROR_SUCCESS) {
    initiator_log_info(INSTANCE_PREFIX "RTL - CS parameters set." LOG_NL, conn_handle);
  }

  initiator_log_info(INSTANCE_PREFIX "RTL - create estimator" LOG_NL, conn_handle);
  rtl_err = sl_rtl_cs_create_estimator(handle);
  if (rtl_err != SL_RTL_ERROR_SUCCESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to create estimator! [err: 0x%02x]" LOG_NL,
                        conn_handle,
                        rtl_err);
    return rtl_err;
  } else {
    initiator_log_info(INSTANCE_PREFIX "RTL - estimator created." LOG_NL, conn_handle);
  }

  return rtl_err;
}

/******************************************************************************
 * Initialize CS configuration.
 *
 * @param[in] conn_handle connection handle
 *****************************************************************************/
static void init_cs_configuration(const uint8_t conn_handle)
{
  sl_status_t sc = SL_STATUS_OK;

  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    on_error(conn_handle,
             CS_ERROR_EVENT_INITIATOR_INSTANCE_NULL,
             SL_STATUS_NULL_POINTER);
    return;
  }
  cs_error_event_t initiator_err = CS_ERROR_EVENT_UNHANDLED;

  initiator_log_info(INSTANCE_PREFIX "CS - set default settings" LOG_NL,
                     initiator->conn_handle);
  sc = sl_bt_cs_set_default_settings(initiator->conn_handle,
                                     sl_bt_cs_role_status_enable,
                                     sl_bt_cs_role_status_disable,
                                     initiator->config.cs_sync_antenna,
                                     initiator->config.max_tx_power_dbm);
  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "set default CS settings failed" LOG_NL,
                        initiator->conn_handle);
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_INITIATOR_FAILED_TO_SET_DEFAULT_CS_SETTINGS,
             sc);
    return;
  }

  initiator_log_debug(INSTANCE_PREFIX "CS - enable security" LOG_NL,
                      initiator->conn_handle);

  sc = sl_bt_cs_security_enable(initiator->conn_handle);
  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "CS - security enable failed! [sc: 0x%lx]" LOG_NL,
                        initiator->conn_handle,
                        (unsigned long)sc);
    initiator_err = CS_ERROR_EVENT_INITIATOR_FAILED_TO_ENABLE_CS_SECURITY;
    on_error(initiator->conn_handle,
             initiator_err,
             sc);
    return;
  } else {
    initiator_log_info(INSTANCE_PREFIX "CS - security enabled." LOG_NL,
                       initiator->conn_handle);
  }

  initiator_log_debug(INSTANCE_PREFIX "CS - create configuration ..." LOG_NL,
                      initiator->conn_handle);
  sc = sl_bt_cs_create_config(initiator->conn_handle,
                              initiator->config.config_id,
                              initiator->config.create_context,
                              initiator->config.cs_mode,
                              initiator->config.submode,
                              initiator->config.min_main_mode_steps,
                              initiator->config.max_main_mode_steps,
                              initiator->config.main_mode_repetition,
                              initiator->config.mode0_step,
                              sl_bt_cs_role_initiator,
                              initiator->config.rtt_type,
                              (cs_phy_t)initiator->config.phy,
                              &initiator->config.channel_map,
                              initiator->config.channel_map_repetition,
                              initiator->config.channel_selection_type,
                              initiator->config.ch3c_shape,
                              initiator->config.ch3c_jump,
                              initiator->config.companion_signal_enable);

  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "CS - configuration create failed! [sc: 0x%lx]" LOG_NL,
                        initiator->conn_handle,
                        (unsigned long)sc);
    initiator_err = CS_ERROR_EVENT_INITIATOR_FAILED_TO_CREATE_CONFIG;
    on_error(initiator->conn_handle, initiator_err, sc);
    return;
  } else {
    initiator_log_info(INSTANCE_PREFIX "CS - configuration created. " LOG_NL,
                       initiator->conn_handle);
  }

  if (sc != SL_STATUS_OK) {
    on_error(initiator->conn_handle, initiator_err, sc);
  }
}

/******************************************************************************
 * Start and restart error timer.
 *****************************************************************************/
static void start_error_timer(const uint8_t conn_handle)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    on_error(conn_handle,
             CS_ERROR_EVENT_INITIATOR_INSTANCE_NULL,
             SL_STATUS_NULL_POINTER);
    return;
  }
  sl_status_t sc = SL_STATUS_OK;
  if (!initiator->error_timer_elapsed) {
    if (initiator->error_timer_started) {
      app_timer_stop(&initiator->timer_handle);
      initiator->error_timer_started = false;
    }
    sc = app_timer_start(&initiator->timer_handle,
                         CS_INITIATOR_ERROR_TIMEOUT_MS,
                         error_timer_cb,
                         (void *)initiator,
                         false);
    if (sc != SL_STATUS_OK) {
      initiator_log_error(INSTANCE_PREFIX "error timer could not start! "
                                          "[sc: 0x%lx]" LOG_NL,
                          initiator->conn_handle,
                          (unsigned long)sc);
      on_error(initiator->conn_handle,
               CS_ERROR_EVENT_TIMER_START_ERROR,
               sc);
    } else {
      initiator_log_debug(INSTANCE_PREFIX "Timer started. [%u ms]" LOG_NL,
                          initiator->conn_handle,
                          CS_INITIATOR_ERROR_TIMEOUT_MS);
      initiator->error_timer_started = true;
    }
  }
}

/******************************************************************************
 * Stop error timer.
 *****************************************************************************/
static void stop_error_timer(const uint8_t conn_handle)
{
  cs_initiator_t *initiator = cs_initiator_get_instance(conn_handle);
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "unknown connection id!" LOG_NL,
                        conn_handle);
    on_error(conn_handle,
             CS_ERROR_EVENT_INITIATOR_INSTANCE_NULL,
             SL_STATUS_NULL_POINTER);
    return;
  }
  sl_status_t sc = SL_STATUS_OK;
  sc = app_timer_stop(&initiator->timer_handle);
  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "could not stop the error timer! "
                                        "[sc: 0x%lx]" LOG_NL,
                        initiator->conn_handle,
                        (unsigned long)sc);
    on_error(initiator->conn_handle,
             CS_ERROR_EVENT_TIMER_STOP_ERROR,
             sc);
    return;
  }
  initiator->error_timer_started = false;
  initiator->error_timer_elapsed = false;
  initiator_log_debug(INSTANCE_PREFIX "Timer stopped." LOG_NL,
                      initiator->conn_handle);
}

// -----------------------------------------------------------------------------
// Public function definitions

/******************************************************************************
 * Init function that sets the first discovery and initiator config
 * and register the selected callback function pointer as a
 * callback for the distance measurement.
 *****************************************************************************/
sl_status_t cs_initiator_create(const uint8_t         conn_handle,
                                cs_initiator_config_t *initiator_config,
                                const rtl_config_t          *rtl_config,
                                cs_result_cb_t        result_cb,
                                cs_intermediate_result_cb_t intermediate_result_cb,
                                cs_error_cb_t         error_cb)
{
  sl_status_t sc = SL_STATUS_OK;
  enum sl_rtl_error_code rtl_err = SL_RTL_ERROR_SUCCESS;
  uint32_t enabled_channels = 0u;
  cs_error_event_t initiator_err = CS_ERROR_EVENT_UNHANDLED;

  if (conn_handle == SL_BT_INVALID_CONNECTION_HANDLE) {
    return SL_STATUS_INVALID_HANDLE;
  }
  if (initiator_config == NULL
      || result_cb == NULL
      || error_cb == NULL
      || intermediate_result_cb == NULL) {
    return SL_STATUS_NULL_POINTER;
  }

  initiator_log_info(INSTANCE_PREFIX "Creating new initiator instance..." LOG_NL, conn_handle);

  cs_initiator_t *initiator =
    cs_initiator_get_free_slot();
  if (initiator == NULL) {
    initiator_log_error(INSTANCE_PREFIX "no more free slots available!" LOG_NL,
                        conn_handle);
    return SL_STATUS_FULL;
  }

  initiator_log_debug(INSTANCE_PREFIX "clean-up initiator and reflector data" LOG_NL,
                      initiator->conn_handle);
  memset(initiator, 0, sizeof(cs_initiator_t));
  initiator->cs_procedure.initiator_subevent_data =
    &initiator->initiator_data.subevent_data[0];
  initiator->cs_procedure.reflector_subevent_data =
    &initiator->reflector_data.subevent_data[0];

  // Assign connection handle
  initiator->conn_handle = conn_handle;
  initiator_log_debug(INSTANCE_PREFIX "load initiator configuration" LOG_NL,
                      initiator->conn_handle);
  memcpy(&initiator->config, initiator_config, sizeof(cs_initiator_config_t));

  #ifndef CS_WEB_ASSEMBLY_MODE
  sc = sl_bt_sm_increase_security(initiator->conn_handle);
  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "failed to increase security" LOG_NL,
                        initiator->conn_handle);
    initiator_err = CS_ERROR_EVENT_INITIATOR_FAILED_TO_INCREASE_SECURITY;
    goto cleanup;
  }
  #endif //CS_WEB_ASSEMBLY_MODE

  // take over RTL lib configuration into the instance
  initiator->rtl_config.algo_mode = rtl_config->algo_mode;
  initiator->rtl_config.cs_parameters = rtl_config->cs_parameters;
  initiator->rtl_config.rtl_logging_enabled = rtl_config->rtl_logging_enabled;

  if (initiator->rtl_config.algo_mode == SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC) {
    initiator_log_info(INSTANCE_PREFIX "RTL - algo mode selected: real-time basic"
                                       "(moving objects tracking)" LOG_NL,
                       initiator->conn_handle);
  } else if (initiator->rtl_config.algo_mode == SL_RTL_CS_ALGO_MODE_STATIC_HIGH_ACCURACY) {
    initiator_log_info(INSTANCE_PREFIX "RTL - algo mode selected: static high accuracy "
                                       "(stationary object tracking)" LOG_NL,
                       initiator->conn_handle);
  } else {
    initiator_log_warning(INSTANCE_PREFIX "unknown algo_mode: %u!"
                                          "Will use the default setting: real-time basic "
                                          "(moving objects tracking)!" LOG_NL,
                          initiator->conn_handle, initiator->rtl_config.algo_mode);
  }

  set_procedure_initial_state(initiator->conn_handle,
                              &initiator->config,
                              &initiator->cs_procedure);

  initiator_log_debug(INSTANCE_PREFIX "initialize discover state machine" LOG_NL,
                      initiator->conn_handle);

  initiator->result_cb = result_cb;
  initiator->intermediate_result_cb = intermediate_result_cb;
  initiator->error_cb = error_cb;
  initiator_log_debug(INSTANCE_PREFIX "registered callbacks" LOG_NL,
                      initiator->conn_handle);

  enabled_channels =
    get_num_tones_from_channel_map(initiator->config.channel_map.data,
                                   initiator->config.channel_map_len);

  initiator_log_info(INSTANCE_PREFIX "CS channel map - channel count: %lu" LOG_NL,
                     initiator->conn_handle,
                     (unsigned long)enabled_channels);

  if (enabled_channels < CS_CHANNEL_MAP_MIN_CHANNELS) {
    initiator_log_error(INSTANCE_PREFIX "CS channel map - too few channels channels enabled!" LOG_NL,
                        initiator->conn_handle);
    sc = SL_STATUS_INVALID_PARAMETER;
    initiator_err = CS_ERROR_EVENT_INITIATOR_CHANNEL_MAP_TOO_FEW_CHANNELS;
    goto cleanup;
  }
  if (enabled_channels > CS_CHANNEL_MAP_MAX_CHANNELS) {
    initiator_log_error(INSTANCE_PREFIX "CS channel - too many channels enabled!" LOG_NL,
                        initiator->conn_handle);
    sc = SL_STATUS_INVALID_PARAMETER;
    initiator_err = CS_ERROR_EVENT_INITIATOR_CHANNEL_MAP_TOO_MANY_CHANNELS;
    goto cleanup;
  }

  #ifndef CS_WEB_ASSEMBLY_MODE
  // Request connection parameter update.
  sc = sl_bt_connection_set_parameters(initiator->conn_handle,
                                       initiator->config.min_connection_interval,
                                       initiator->config.max_connection_interval,
                                       initiator->config.latency,
                                       initiator->config.timeout,
                                       initiator->config.min_ce_length,
                                       initiator->config.max_ce_length);
  if (sc != SL_STATUS_OK) {
    initiator_log_error(INSTANCE_PREFIX "CS - failed to set connection parameters! "
                                        "[sc: 0x%04lx]" LOG_NL,
                        initiator->conn_handle,
                        (unsigned long)sc);
    initiator_err =
      CS_ERROR_EVENT_INITIATOR_FAILED_TO_SET_CONNECTION_PARAMETERS;
    goto cleanup;
  }
  #endif //CS_WEB_ASSEMBLY_MODE

  initiator_log_debug(INSTANCE_PREFIX "CS - set connection parameters ..." LOG_NL,
                      initiator->conn_handle);
  initiator->discovery_state.characteristic_found.ras_control_point = false;
  initiator->discovery_state.characteristic_found.ras_procedure_enable_data = false;
  initiator->discovery_state.characteristic_found.ras_subevent_ranging_data = false;
  initiator->discovery_state.characteristic_found.ras_all = false;

  start_error_timer(initiator->conn_handle);
  // trying to initialize RTL lib within error-timeout
  initiator_log_debug(INSTANCE_PREFIX "RTL - initialize lib item" LOG_NL,
                      initiator->conn_handle);
  rtl_err = rtl_library_init(initiator->conn_handle,
                             &initiator->rtl_handle,
                             &initiator->rtl_config,
                             initiator->config.cs_mode);
  if (rtl_err != SL_RTL_ERROR_SUCCESS) {
    initiator_log_error(INSTANCE_PREFIX "RTL - failed to init lib item! [err: 0x%02x]" LOG_NL,
                        initiator->conn_handle,
                        rtl_err);
    initiator_err = CS_ERROR_EVENT_INITIATOR_FAILED_TO_INIT_RTL_LIB;
  }
  stop_error_timer(initiator->conn_handle);
  initiator_log_info(INSTANCE_PREFIX "RTL - lib item initialized." LOG_NL,
                     initiator->conn_handle);
  (void)initiator_state_machine_event_handler(initiator->conn_handle,
                                              INITIATOR_EVT_INIT_STARTED,
                                              NULL);
  cleanup:
  if (sc != SL_STATUS_OK) {
    on_error(initiator->conn_handle, initiator_err, sc);
  }
  return sc;
}

/******************************************************************************
 * Initialize instance slots.
 *****************************************************************************/
void cs_initiator_init(void)
{
  for (uint8_t i = 0u; i < CS_INITIATOR_MAX_CONNECTIONS; i++) {
    cs_initiator_t *initiator = &cs_initiator_instances[i];
    memset(initiator, 0, sizeof(cs_initiator_instances[0]));
    initiator->conn_handle = SL_BT_INVALID_CONNECTION_HANDLE;
    initiator->initiator_state = INITIATOR_STATE_UNINITIALIZED;
    // Assign subevent data array start address to cs_procedure subevent data
    // pointer element.
    initiator->cs_procedure.initiator_subevent_data =
      &initiator->initiator_data.subevent_data[0];
    initiator->cs_procedure.reflector_subevent_data =
      &initiator->reflector_data.subevent_data[0];
  }
}

/******************************************************************************
 * Delete existing initiator instance after closing its connection to
 * reflector(s).
 *****************************************************************************/
sl_status_t cs_initiator_delete(const uint8_t conn_handle)
{
  sl_status_t sc;
  (void)cs_initiator_get_instance(conn_handle);
  if (conn_handle == SL_BT_INVALID_CONNECTION_HANDLE) {
    return SL_STATUS_INVALID_HANDLE;
  }
  sc = initiator_state_machine_event_handler(conn_handle,
                                             INITIATOR_EVT_DELETE_INSTANCE,
                                             NULL);
  return sc;
}

/******************************************************************************
 * Deinitialization function of CS Initiator.
 * Delete initiator for all existing instances.
 *****************************************************************************/
void cs_initiator_deinit(void)
{
  for (uint8_t i = 0u; i < CS_INITIATOR_MAX_CONNECTIONS; i++) {
    cs_initiator_t *initiator = &cs_initiator_instances[i];
    if (initiator->conn_handle != SL_BT_INVALID_CONNECTION_HANDLE) {
      cs_initiator_delete(initiator->conn_handle);
    }
  }
}

// -----------------------------------------------------------------------------
// Event / callback definitions

/******************************************************************************
 * Bluetooth stack event handler.
 *****************************************************************************/
bool cs_initiator_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  cs_initiator_t *initiator;
  state_machine_event_data_t evt_data;
  bool handled = false;

  switch (SL_BT_MSG_ID(evt->header)) {
    // --------------------------------
    // Connection parameters set
    case sl_bt_evt_connection_parameters_id:
      initiator = cs_initiator_get_instance(evt->data.evt_connection_parameters.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_connection_parameters_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_connection_parameters.connection);
        break;
      }

      if (evt->data.evt_connection_parameters.security_mode
          != sl_bt_connection_mode1_level1) {
        initiator_log_info(INSTANCE_PREFIX "CS - connection parameters set: "
                                           "encryption on. Start discovering RAS service "
                                           "& characteristic ..." LOG_NL,
                           initiator->conn_handle);

        sc = sl_bt_gatt_discover_primary_services_by_uuid(initiator->conn_handle,
                                                          UUID_LEN,
                                                          ras_service_uuid);
        if (sc != SL_STATUS_OK && sc != SL_STATUS_IN_PROGRESS) {
          initiator_log_error(INSTANCE_PREFIX "failed to start RAS service discovery" LOG_NL,
                              initiator->conn_handle);
          on_error(initiator->conn_handle,
                   CS_ERROR_EVENT_START_SERVICE_DISCOVERY,
                   sc);

          break;
        }

        init_cs_configuration(initiator->conn_handle);
      }
      break;

    // --------------------------------
    // CS Security
    case sl_bt_evt_cs_security_enable_complete_id:
      initiator = cs_initiator_get_instance(evt->data.evt_cs_security_enable_complete.connection);
      if (initiator == NULL) {
        break;
      }
      initiator->cs_security_enabled = true;
      initiator_log_info(INSTANCE_PREFIX "CS security elevated" LOG_NL,
                         initiator->conn_handle);
      break;

    // --------------------------------
    // GATT procedure completed
    case sl_bt_evt_gatt_procedure_completed_id:
      initiator = cs_initiator_get_instance(evt->data.evt_gatt_procedure_completed.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_gatt_procedure_completed_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_connection_parameters.connection);
        break;
      }
      handled = gatt_procedure_complete_event(initiator->conn_handle, evt);
      break;

    // --------------------------------
    // New GATT characteristic discovered
    case sl_bt_evt_gatt_characteristic_id:
      initiator = cs_initiator_get_instance(evt->data.evt_gatt_characteristic.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_gatt_characteristic_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_gatt_service.connection);
        break;
      }
      initiator_log_debug(INSTANCE_PREFIX "GATT - discovered new characteristic" LOG_NL,
                          initiator->conn_handle);
      handled = check_characteristic_uuid(initiator->conn_handle, evt);
      break;

    // --------------------------------
    // New GATT service discovered
    case sl_bt_evt_gatt_service_id:
      initiator = cs_initiator_get_instance(evt->data.evt_gatt_service.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event [sl_bt_evt_gatt_service_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_gatt_service.connection);
        break;
      }
      if (evt->data.evt_gatt_service.uuid.len == UUID_LEN) {
        if (memcmp(ras_service_uuid, evt->data.evt_gatt_service.uuid.data, UUID_LEN) == 0) {
          initiator->ras_service_handle = evt->data.evt_gatt_service.service;
          initiator->discovery_state.action = CS_TRANSPORT_START_DISCOVERING_RAS_CHARACTERISTICS;
          initiator_log_debug(INSTANCE_PREFIX "RAS - found service" LOG_NL,
                              initiator->conn_handle);
          start_error_timer(initiator->conn_handle);
        }
      }
      break;

    // --------------------------------
    // New GATT characteristic value received
    case sl_bt_evt_gatt_characteristic_value_id:
      initiator = cs_initiator_get_instance(evt->data.evt_gatt_characteristic_value.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_gatt_characteristic_value_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_gatt_characteristic_value.connection);
        break;
      }
      if ((evt->data.evt_gatt_characteristic_value.characteristic
           == initiator->characteristic_handle.ras_control_point)
          && (evt->data.evt_gatt_characteristic_value.att_opcode
              == sl_bt_gatt_handle_value_indication)) {
        initiator_log_info(INSTANCE_PREFIX "RAS - received new \'Control Point\' data" LOG_NL,
                           initiator->conn_handle);
        handled = true;

        ras_ranging_data_get_command_t cmd;
        sc = cs_ras_create_control_point_response(evt->data.evt_gatt_characteristic_value.value.data,
                                                  evt->data.evt_gatt_characteristic_value.value.len,
                                                  CONFIG_RAS_REQUEST_PERIODIC_NOTIFICATION_MODE,
                                                  &cmd);
        if (sc != SL_STATUS_OK) {
          initiator_log_error(INSTANCE_PREFIX "RAS - failed to create "
                                              "\'Control Point\' response" LOG_NL,
                              initiator->conn_handle);
          on_error(initiator->conn_handle,
                   CS_ERROR_EVENT_SEND_CONTROL_POINT_RESPONSE_FAILED,
                   sc);
          break;
        }
        sc = sl_bt_gatt_write_characteristic_value(initiator->conn_handle,
                                                   initiator->characteristic_handle.ras_control_point,
                                                   sizeof(ras_ranging_data_get_command_t),
                                                   (uint8_t*)&cmd);
        if (sc != SL_STATUS_OK) {
          initiator_log_error(INSTANCE_PREFIX "RAS - failed to write characteristic value" LOG_NL,
                              initiator->conn_handle);
          on_error(initiator->conn_handle,
                   CS_ERROR_EVENT_WRITE_CHARACTERISTIC_FAILED,
                   sc);
          break;
        }

        sc = sl_bt_gatt_send_characteristic_confirmation(initiator->conn_handle);
        if (sc != SL_STATUS_OK) {
          initiator_log_error(INSTANCE_PREFIX "RAS - failed to send "
                                              "\'Control \'Point confirmation" LOG_NL,
                              initiator->conn_handle);
          on_error(initiator->conn_handle,
                   CS_ERROR_EVENT_SEND_CHARACTERISTIC_CONFIRMATION_FAILED,
                   sc);
          break;
        }
      }

      if (evt->data.evt_gatt_characteristic_value.characteristic
          != initiator->characteristic_handle.ras_subevent_ranging_data) {
        break;
      }

      if (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_notification) {
        handled = true;
        struct sl_bt_evt_cs_result_s *result_ptr =
          (struct sl_bt_evt_cs_result_s*)evt->data.evt_gatt_characteristic_value.value.data;

        // Extract CS result data and process it
        evt_data.evt_cs_result.cs_result_data = result_ptr;
        evt_data.evt_cs_result.initiator_part = false;
        (void)initiator_state_machine_event_handler(evt->data.evt_cs_result.connection,
                                                    INITIATOR_EVT_CS_RESULT,
                                                    &evt_data);
      }
      break;

    // --------------------------------
    // CS procedure enable action completed
    case sl_bt_evt_cs_procedure_enable_complete_id:
      initiator = cs_initiator_get_instance(evt->data.evt_cs_procedure_enable_complete.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_cs_procedure_enable_complete_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_cs_procedure_enable_complete.connection);
        break;
      }
      handled = true;
      evt_data.evt_procedure_enable_completed = &evt->data.evt_cs_procedure_enable_complete;
      if (evt->data.evt_cs_procedure_enable_complete.state) {
        // procedure enable/disable received
        (void)initiator_state_machine_event_handler(evt->data.evt_cs_procedure_enable_complete.connection,
                                                    INITIATOR_EVT_PROCEDURE_ENABLE_COMPLETED,
                                                    &evt_data);
      }
      break;

    // --------------------------------
    // CS configuration completed
    case sl_bt_evt_cs_config_complete_id:
      initiator = cs_initiator_get_instance(evt->data.evt_cs_config_complete.connection);
      if (initiator == NULL) {
        initiator_log_error("Unexpected event "
                            "[sl_bt_evt_cs_config_complete_id]!"
                            "Unknown target connection id: %u" LOG_NL,
                            evt->data.evt_cs_config_complete.connection);
        break;
      } else {
        initiator_log_info(INSTANCE_PREFIX "CS - configuration completed. "
                                           "Set CS procedure parameters ..." LOG_NL,
                           initiator->conn_handle);

        stop_error_timer(initiator->conn_handle);

        initiator->cs_procedure.cs_config.ch3c_jump = evt->data.evt_cs_config_complete.ch3c_jump;
        initiator->cs_procedure.cs_config.ch3c_shape = evt->data.evt_cs_config_complete.ch3c_shape;
        initiator->cs_procedure.cs_config.channel_map_repetition = evt->data.evt_cs_config_complete.channel_map_repetition;
        initiator->cs_procedure.cs_config.cs_sync_phy = evt->data.evt_cs_config_complete.cs_sync_phy;
        initiator->cs_procedure.cs_config.rtt_type = evt->data.evt_cs_config_complete.rtt_type;
        initiator->cs_procedure.cs_config.num_calib_steps = evt->data.evt_cs_config_complete.mode_calibration_steps;
        initiator->cs_procedure.cs_config.T_FCS_time = evt->data.evt_cs_config_complete.fcs_time_us;
        initiator->cs_procedure.cs_config.T_IP1_time = evt->data.evt_cs_config_complete.ip1_time_us;
        initiator->cs_procedure.cs_config.T_IP2_time = evt->data.evt_cs_config_complete.ip2_time_us;
        initiator->cs_procedure.cs_config.T_PM_time = evt->data.evt_cs_config_complete.pm_time_us;

        sc = sl_bt_cs_set_procedure_parameters(initiator->conn_handle,
                                               initiator->config.config_id,
                                               initiator->config.max_procedure_duration,
                                               initiator->config.min_procedure_interval,
                                               initiator->config.max_procedure_interval,
                                               initiator->config.max_procedure_count,
                                               initiator->config.min_subevent_len,
                                               initiator->config.max_subevent_len,
                                               initiator->config.antenna_config,
                                               initiator->config.phy,
                                               initiator->config.tx_pwr_delta,
                                               initiator->config.preferred_peer_antenna,
                                               initiator->config.snr_control_initiator,
                                               initiator->config.snr_control_reflector);

        if (sc != SL_STATUS_OK) {
          initiator_log_error(INSTANCE_PREFIX "CS procedure - failed to set parameters! "
                                              "[sc: 0x%lx]" LOG_NL,
                              initiator->conn_handle,
                              (unsigned long)sc);
          on_error(initiator->conn_handle,
                   CS_ERROR_EVENT_CS_SET_PROCEDURE_PARAMETERS_FAILED,
                   sc);
          break;
        }
      }
      handled = true;
      break;

    // --------------------------------
    // CS result (initiator) arrived
    case sl_bt_evt_cs_result_id:
      initiator_log_info(INSTANCE_PREFIX "CS - received initator CS result" LOG_NL,
                         evt->data.evt_cs_result.connection);

      evt_data.evt_cs_result.cs_result_data = &evt->data.evt_cs_result;
      evt_data.evt_cs_result.initiator_part = true;
      sc = initiator_state_machine_event_handler(evt->data.evt_cs_result.connection,
                                                 INITIATOR_EVT_CS_RESULT,
                                                 &evt_data);
      if (sc == SL_STATUS_OK) {
        handled = true;
      }
      break;

    // --------------------------------
    // Bluetooth stack resource exhausted
    case sl_bt_evt_system_resource_exhausted_id:
    {
      uint8_t num_buffers_discarded =
        evt->data.evt_system_resource_exhausted.num_buffers_discarded;
      uint8_t num_buffer_allocation_failures =
        evt->data.evt_system_resource_exhausted.num_buffer_allocation_failures;
      uint8_t num_heap_allocation_failures =
        evt->data.evt_system_resource_exhausted.num_heap_allocation_failures;
      initiator_log_error("BT stack buffers exhausted, "
                          "data loss may have occurred! [buf_discarded:%u, "
                          "buf_alloc_fail:%u, heap_alloc_fail:%u]" LOG_NL,
                          num_buffers_discarded,
                          num_buffer_allocation_failures,
                          num_heap_allocation_failures);
    }
    break;

    default:
      initiator_log_debug("unhandled BLE event: 0x%lx" LOG_NL,
                          (unsigned long)SL_BT_MSG_ID(evt->header));
      break;
  }

  // Return false if the event was handled above.
  return !handled;
}

/******************************************************************************
 * Weak implementation of the on_waiting indication for CS Initiator.
 *****************************************************************************/
SL_WEAK void cs_initiator_on_waiting(void)
{
}
