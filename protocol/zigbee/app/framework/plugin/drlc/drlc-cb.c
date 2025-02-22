/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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

#include "app/framework/include/af.h"

/** @brief Event Action
 *
 * This function is called by the demand response and load control client
 * plugin whenever an event status changes within the DRLC event table. The
 * list of possible event status values is defined by the ZCL spec and is
 * listed in the Application Framework's generated enums located in enums.h.
 * For example, an event status may be:
 * AMI_EVENT_STATUS_LOAD_CONTROL_EVENT_COMMAND_RX indicating that a properly
 * formatted event was received; AMI_EVENT_STATUS_EVENT_STARTED indicating that
 * an event has started; AMI_EVENT_STATUS_THE_EVENT_HAS_BEEN_CANCELED,
 * indicating that the event was canceled. This callback is intended to give
 * the device an opportunity to take action on the event in question. For
 * instance if an event starts, the device should take the appropriate event
 * action on the hardware. This callback returns a bool, if returned value is
 * true, then a notification will be send over the air automatically to the
 * originator of the event. If it is false, then nothing will be sent back to
 * the originator of the event. Please note that in order for your application
 * to be ZigBee compliant, a notification must be sent over the air to the
 * originator of the event, so a value of false should only be returned if your
 * application code takes care of sending this message or there is some other
 * reason a message does not need to be sent by the framework.
 *
 * @param loadControlEvent Actual event Ver.: always
 * @param eventStatus Status of event Ver.: always
 * @param sequenceNumber Sequence number Ver.: always
 */
WEAK(bool sl_zigbee_af_drlc_event_action_cb(sl_zigbee_af_load_control_event_t *loadControlEvent,
                                            sl_zigbee_af_ami_event_status_t eventStatus,
                                            uint8_t sequenceNumber))
{
  return true;
}
