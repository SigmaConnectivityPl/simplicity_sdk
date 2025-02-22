/***************************************************************************//**
 * @file sl_zigbee_dynamic_commissioning_ipc_callback_events.h
 * @brief callback struct and event handlers for sl_zigbee_dynamic_commissioning
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
// automatically generated from sl_zigbee_dynamic_commissioning.h.  Do not manually edit
#ifndef SL_ZIGBEE_DYNAMIC_COMMISSIONING_IPC_CALLBACK_EVENTS_H
#define SL_ZIGBEE_DYNAMIC_COMMISSIONING_IPC_CALLBACK_EVENTS_H

#include "stack/internal/inc/sl_zigbee_dynamic_commissioning_internal_def.h"

typedef struct {
  sl_zigbee_address_info ids;
  sl_zigbee_dynamic_commissioning_event_t event;
} sli_zigbee_stack_dynamic_commissioning_alert_callback_ipc_event_t;

#endif // SL_ZIGBEE_DYNAMIC_COMMISSIONING_IPC_CALLBACK_EVENTS_H
