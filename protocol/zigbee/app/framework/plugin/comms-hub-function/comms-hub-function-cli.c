/***************************************************************************//**
 * @file
 * @brief CLI for the Comms Hub Function plugin.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
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
#include "comms-hub-function.h"
#include "sleepy-message-queue-config.h"
#include "app/framework/util/af-main.h"
#include "comms-hub-function.h"
#include "comms-hub-tunnel-endpoints.h"
#include "tunnel-manager.h"
#include "app/framework/plugin/gbz-message-controller/gbz-message-controller.h"

typedef struct {
  uint16_t messageCode;
  const char *  description;
} sli_zigbee_gbcs_use_case_description_t;

static sli_zigbee_gbcs_use_case_description_t useCaseDescriptions[] = {
  { GCS05_MESSAGE_CODE, "GCS05 Update Prepayment Configurations on GSME" },
  { GCS06_MESSAGE_CODE, "GCS06 Activate Emergency Credit Remotely on GSME" },
  { GCS07_MESSAGE_CODE, "GCS07 Send Message to GSME" },
  { GCS11_MESSAGE_CODE, "GCS11 Disable Privacy PIN Protection on GSME" },
  { GCS23_MESSAGE_CODE, "GCS23 Set CV and Conversion Factor Value(s) on the GSME" },
  { GCS44_MESSAGE_CODE, "GCS44 Write Contact Details on GSME" },
  { GCS01b_MESSAGE_CODE, "GCS01b Set Price on GSME" },
};
#define GBCS_NUM_USE_CASES (sizeof(useCaseDescriptions) / sizeof(useCaseDescriptions[0]))

#define SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_MSG_CACHE   (3)
#define SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_SEND_LENGTH (350)

static uint8_t messagePayload[SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_MSG_CACHE][SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_SEND_LENGTH];
static uint8_t nextMessage = 0;

// Prototypes
static void sendMessage(sl_802154_long_addr_t deviceId, uint16_t length, uint8_t *message, bool includeHeader, uint16_t messageCode);
void sli_zigbee_af_comms_hub_function_cli_print_supported_use_cases(sl_cli_command_arg_t *arguments);

// Internal Functions
static uint16_t findUseCaseDescription(uint16_t messageCode)
{
  uint16_t index;

  for (index = 0; index < GBCS_NUM_USE_CASES; index++) {
    if (useCaseDescriptions[index].messageCode == messageCode) {
      return index;
    }
  }

  return index;
}

// External Functions

// plugin comms-hub-function simulate-gbz-msg <uint16_t:messageCode>
void sli_zigbee_af_comms_hub_function_cli_simulate_gbz_msg(sl_cli_command_arg_t *arguments)
{
  sl_802154_long_addr_t destinationDeviceId;
  uint16_t messageCode = sl_cli_get_argument_uint16(arguments, 1);
  uint32_t currentTime = sl_zigbee_af_get_current_time();
  uint16_t index = findUseCaseDescription(messageCode);

  sl_zigbee_copy_eui64_arg(arguments, 0, destinationDeviceId, true);

  if (index >= GBCS_NUM_USE_CASES) {
    sl_zigbee_af_comms_hub_function_println("Unsupported message code: 0x%2x", messageCode);
    sli_zigbee_af_comms_hub_function_cli_print_supported_use_cases(arguments);
    return;
  }

  sl_zigbee_af_comms_hub_function_println("GBCS Use Case: %p", useCaseDescriptions[index].description);
  sl_zigbee_af_comms_hub_function_println("Current Time: 0x%4x", currentTime);

  if (messageCode == GCS05_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x0B, // component count - 8-14

                             // only testing simple cmds for now.
                             // component #0 - Emergency Credit Setup
                             0x00,  // extended header control field
                             0x07, 0x05,  // extended header cluster id
                             0x00, 0x13,  // extended gbz command length
                             0x01,  // frame control
                             0x00,  // trans. seq number
                             0x03,  // ZCL command id
                             0x00, 0x00, 0x00, 0x00,  // issuerEventId - cur utc time // index-11
                             0x00, 0x00, 0x00, 0x00,  // startTime - now
                             0x00, 0x00, 0x00, 0x00,  // emergency credit limit
                             0x00, 0x00, 0x00, 0x00,  // emergency credit threshold

                             // component #1 - Set Overall Debt Cap
                             0x00,  // extended header control field
                             0x07, 0x05,  // extended header cluster id
                             0x00, 0x13,  // extended gbz command length
                             0x01,  // frame control
                             0x01,  // trans. seq number
                             0x0C,  // ZCL command id
                             0x00, 0x00, 0x00, 0x00,  // provider id - 0x00
                             0x00, 0x00, 0x00, 0x00,  // issuer event id - cur utc time // index-39
                             0x00, 0x00, 0x00, 0x00,  // implementation date/time - now
                             0x44, 0x33, 0x22, 0x11,  // OverallDebtCap

                             // component #2 - Set Low Credit Warning Level
                             0x00,  // extended header control field
                             0x07, 0x05,  // extended header cluster id
                             0x00, 0x07,  // extended gbz command length
                             0x01,  // frame control
                             0x02,  // trans. seq number
                             0x09,  // ZCL command id
                             0x44, 0x33, 0x22, 0x11,  // low credit warning level

                             // component #3 - Set Maximum Credit Limit
                             0x00,  // extended header control field
                             0x07, 0x05,  // extended header cluster id
                             0x00, 0x17,  // extended gbz command length
                             0x01,  // frame control
                             0x03,  // trans. seq number
                             0x0B,  // ZCL command id
                             0x00, 0x00, 0x00, 0x00,  // provider id - unused
                             0x00, 0x00, 0x00, 0x00,  // issuer event id - // index-75
                             0x00, 0x00, 0x00, 0x00,  // implementation date/time -now
                             0x44, 0x33, 0x22, 0x11,  // max meter balance
                             0x00, 0x33, 0x22, 0x11,  // largest value of any one credit

                             // component #4 - PublishCalendar
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x19,  // extended gbz command length
                             0x09,  // frame control
                             0x04,  // trans. seq number
                             0x00,  // command id - PublishCalendar
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-103
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-107
                             0x00, 0x00, 0x00, 0x00,  // start time - now
                             0x03,  // calendar type - 3 / Friendly Credit Calendar
                             0x00,  // calendarTimeRef - 0x00 - UTC time
                             0x00,  // calendar name
                             0x04,  // number of seasons
                             0x02,  // number of week profiles
                             0x02,  // number of day profiles

                             // component #5 - PublishDayProfile
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x1D,  // extended gbz command length
                             0x09,  // frame control
                             0x05,  // trans. seq number
                             0x01,  // command id - PublishDayProfile
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-133
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-137
                             0x01,   // day id
                             0x03,  // total number of schedule entries
                             0x00,  // command index
                             0x01,  // total number of commands
                             0x03,  // calendar type - 3 / Friendly Credit Calendar
                             0x00, 0x00, 0x01,  //day sch entry 0 - 12:00 AM, tier 1
                             0xE0, 0x01, 0x02,  //day sch entry 1 - 08:00 AM, tier 2
                             0xC0, 0x03, 0x03,  //day sch entry 2 - 04:00 PM, tier 3

                             // component #6 - PublishDayProfile
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x1D,  // extended gbz command length
                             0x09,  // frame control
                             0x06,  // trans. seq number
                             0x01,  // command id - PublishDayProfile
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-167
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-171
                             0x02,  // day id
                             0x03,  // total number of schedule entries
                             0x00,  // command index
                             0x01,  // total number of commands
                             0x03,  // calendar type - 3 / Friendly Credit Calendar
                             0x00, 0x00, 0x04,  //day sch entry 0 - 12:00 AM, tier 4
                             0xE0, 0x01, 0x05,  //day sch entry 1 - 08:00 AM, tier 5
                             0xC0, 0x03, 0x06,  //day sch entry 2 - 04:00 PM, tier 6

                             // component #7 - PublishWeekProfile
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x17,  // extended gbz command length
                             0x09,  // frame control
                             0x07,  // trans. seq number
                             0x02,  // command id - PublishWeekProfile
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-201
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-205
                             0x01,  // week id
                             0x01,  // mon
                             0x01,  // tue
                             0x01,  // wed
                             0x01,  // thu
                             0x01,  // fri
                             0x01,  // sat
                             0x01,  // sun

                             // component #8 - PublishWeekProfile
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x17,  // extended gbz command length
                             0x09,  // frame control
                             0x08,  // trans. seq number
                             0x02,  // command id - PublishWeekProfile
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-229
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-233
                             0x02,  // week id
                             0x02,  // mon
                             0x02,  // tue
                             0x02,  // wed
                             0x02,  // thu
                             0x02,  // fri
                             0x02,  // sat
                             0x02,  // sun

                             // component #9 - PublishSeasons
                             0x00,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x25,  // extended gbz command length
                             0x09,  // frame control
                             0x09,  // trans. seq number
                             0x03,  // command id - PublishSeasons
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-257
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-261
                             0x00,  // command index
                             0x01,  // number of commands
                             0x75, 0x01, 0x01, 0x00, 0x01,  // season entry 0 - 01/01/2017, wk 1
                             0x75, 0x04, 0x01, 0x00, 0x02,  // season entry 0 - 04/01/2017, wk 2
                             0x75, 0x07, 0x01, 0x00, 0x02,  // season entry 0 - 07/01/2017, wk 2
                             0x75, 0x0A, 0x01, 0x00, 0x01,  // season entry 0 - 10/01/2017, wk 1

                             // component #10 - PublishSpecialDays
                             0x01,  // extended header control field
                             0x07, 0x07,  // extended header cluster id
                             0x00, 0x1C,  // extended gbz command length
                             0x09,  // frame control
                             0x0A,  // trans. seq number
                             0x04,  // command id - PublishSpecialDays
                             0x00, 0x00, 0x00, 0x00,  // provider id
                             0x00, 0x00, 0x00, 0x00,  // issuer event id // index-299
                             0x00, 0x00, 0x00, 0x00,  // issuer calendar id // index-303
                             0x00, 0x00, 0x00, 0x00,  // start time
                             0x03,  // calendar type - Friendly Credit Calendar
                             0x01,  // total number of specialdays
                             0x00,  // Command Index
                             0x01,  // total number of commands
                             0x75, 0x06, 0x01, 0x00, 0x01,  // special day - 06/01/2017, ref day id - 1
    };

    sl_zigbee_af_copy_int32u(gbzCommand, 11, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 39, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 75, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 103, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 107, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 133, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 137, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 167, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 171, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 201, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 205, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 229, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 233, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 257, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 261, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 299, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 303, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS06_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x01, // component count

                             // component #1
                             0x01, // extended header control field
                             0x07, 0x05, // extended header cluster id
                             0x00, 0x08, // extended gbz command length
                             0x01, // frame control
                             0x00, // trans. seq number
                             0x00, // ZCL command id
                             0x00, 0x00, 0x00, 0x00, // Date and time command is issued
                             0x00, // Source: 0x00=Energy Service Interface
    };

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS07_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x01, // component count

                             // component #1
                             0x01, // extended header control field
                             0x07, 0x03, // extended header cluster id
                             0x00, 0x12, // extended gbz command length
                             0x09, // frame control
                             0x00, // trans. seq number
                             0x00, // ZCL command id - Display Message
                             0x00, 0x00, 0x00, 0x00, // msg id - curtime
                             0x00, // msg control - normal transmission
                             0x00, 0x00, 0x00, 0x00, // start time - now
                             0xFF, 0xFF, // duration in minutes - until changed
                             0x03, // msg length
                             0x41, 0x42, 0x43 // msg = "ABC"
    };
    sl_zigbee_af_copy_int32u(gbzCommand, 11, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS11_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x01, // component count

                             // component #1
                             0x01, // extended header control field
                             0x07, 0x08, // extended header cluster id
                             0x00, 0x0F, // extended gbz command length
                             0x09, // frame control
                             0x00, // trans. seq number
                             0x02, // ZCL command id - Display Message
                             0x00, 0x00, 0x00, 0x00, // issuer id - curtime
                             0x00, 0x00, 0x00, 0x00, // start time - now
                             0x00, 0x00, // PIN is valid until changed
                             0x02, // 2 = access to the consumer menu
                             0x00, //Password, 0-length string
    };
    sl_zigbee_af_copy_int32u(gbzCommand, 11, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS23_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x02, // component count

                             // component #1
                             0x00, // extended header control field
                             0x07, 0x00, // extended header cluster id
                             0x00, 0x11, // extended gbz command length
                             0x09, // frame control
                             0x00, // trans. seq number
                             0x03, // ZCL command id
                             0x00, 0x00, 0x00, 0x00, // issuer event id - curtime
                             0x00, 0x00, 0x00, 0x00, // current time - curtime
                             0x04, 0x03, 0x02, 0x01, // calorific value
                             0x01, // calorific value unit - MJ/m3
                             0x10, // calorific value trailing digit

                             // component #2
                             0x01, // extended header control field
                             0x07, 0x00, // extended header cluster id
                             0x00, 0x10, // extended gbz command length
                             0x09, // frame control
                             0x01, // trans. seq number
                             0x02, // ZCL command id
                             0x00, 0x00, 0x00, 0x00, // issuer event id - curtime
                             0x00, 0x00, 0x00, 0x00, // current time - curtime
                             0x0D, 0x0C, 0x0B, 0x0A, // conversion factor -
                             0x50, // conversion factor trailing digit
    };
    sl_zigbee_af_copy_int32u(gbzCommand, 11, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 33, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS44_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x01, // component count

                             // component #1
                             0x01, // extended header control field
                             0x07, 0x08, // extended header cluster id
                             0x00, 0x1C, // extended gbz command length
                             0x09, // frame control
                             0x00, // trans. seq number
                             0x01, // ZCL command id - ChangeOfSupplier
                             0x00, 0x00, 0x00, 0x00, // current provider id
                             0x00, 0x00, 0x00, 0x00, // issuer event id - curtime
                             0x00, // tariffType - 0
                             0x00, 0x00, 0x00, 0x00, // proposed provider id
                             0x00, 0x00, 0x00, 0x00, // proposed change implementation time - curtime
                             0x00, 0x00, 0x00, 0x00, // provider change control
                             0x01, 0xAA, // proposed provider name
                             0x01, 0xBB // proposed provider contact details
    };
    sl_zigbee_af_copy_int32u(gbzCommand, 15, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 24, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  } else if (messageCode == GCS01b_MESSAGE_CODE) {
    uint8_t gbzCommand[] = { 0x01, 0x09, // profile id
                             0x02, // component count

                             // component #1
                             0x00, // extended header control field
                             0x07, 0x00, // extended header cluster id
                             0x00, 0x27, // extended gbz command length
                             0x09, // frame control
                             0x00, // trans. seq number
                             0x04, // ZCL command id - PublishTariffInfo
                             0x00, 0x00, 0x00, 0x00, // provider id
                             0x00, 0x00, 0x00, 0x00, // issuer event id - curtime
                             0x01, 0x00, 0x00, 0x00, // issuer tariff id - tariff switching table
                             0x00, 0x00, 0x00, 0x00, // starttime - 0
                             0x00, // tarifftype - 0
                             0x01, 0xAA, // tariff label - 0
                             0x01, // number of price tiers in use
                             0x01, // number of block tiers in use
                             0x00, // unit of measurement
                             0x0A, 0x33, // currency - GBP
                             0x00, // price trailing digit
                             0x00, 0x00, 0x00, 0x00, // standing charge
                             0xFF, // tier block mode
                             0x00, 0x00, 0x00, // block threshold multiplier
                             0x00, 0x00, 0x00, // block threshold divisor

                             // component #2 - PublishPriceMatrix
                             0x01, // extended header control field
                             0x07, 0x00, // extended header cluster id
                             0x00, 0x16, // extended gbz command length
                             0x09, // frame control
                             0x01, // trans. seq number
                             0x05, // ZCL command id - PublishPriceMatrix
                             0x00, 0x00, 0x00, 0x00, // provider id - unused
                             0x00, 0x00, 0x00, 0x00, // issuer event id - curtime
                             0x00, 0x00, 0x00, 0x00, // start time - 0
                             0x01, 0x00, 0x00, 0x00, // issuer tariff id - 1, tariff switching table
                             0x00, // command index
                             0x01, // command index
                             0x00, // sub-payload control
                             0x01, // tier block id
                             0x04, 0x03, 0x02, 0x01 // tier block id price
    };
    sl_zigbee_af_copy_int32u(gbzCommand, 14, currentTime);
    sl_zigbee_af_copy_int32u(gbzCommand, 58, currentTime);

    sl_zigbee_af_comms_hub_function_print("GBZ Command: ");
    sl_zigbee_af_comms_hub_function_print_buffer(gbzCommand, sizeof(gbzCommand), true);
    sl_zigbee_af_comms_hub_function_println("");

    sendMessage(destinationDeviceId, sizeof(gbzCommand), gbzCommand, true, messageCode);
  }
}

void sli_zigbee_af_comms_hub_function_cli_print_supported_use_cases(sl_cli_command_arg_t *arguments)
{
  uint16_t index;

  sl_zigbee_af_comms_hub_function_println("Supported Use Cases");
  for (index = 0; index < GBCS_NUM_USE_CASES; index++) {
    sl_zigbee_af_comms_hub_function_println("Message Code: 0x%2x, GBCS Use Case: %p",
                                            useCaseDescriptions[index].messageCode,
                                            useCaseDescriptions[index].description);
  }
}

void sli_zigbee_af_comms_hub_function_cli_send(sl_cli_command_arg_t *arguments)
{
  sl_802154_long_addr_t deviceId;
  uint8_t length;
  uint8_t message[255];
  uint8_t* command = sl_zigbee_cli_get_argument_string_and_length(arguments, -1, &length);

  sl_zigbee_copy_eui64_arg(arguments, 0, deviceId, true);
  if (command[5] == 's') {
    length = sl_zigbee_copy_string_arg(arguments,
                                       1,
                                       message,
                                       255, false);
  } else {
    length = sl_zigbee_copy_hex_arg(arguments,
                                    1,
                                    message,
                                    255, false);
  }
  sendMessage(deviceId, length, message, false, TEST_MESSAGE_CODE);
}

void sli_zigbee_af_comms_hub_function_cli_timeout(sl_cli_command_arg_t *arguments)
{
  uint32_t timeout = sl_cli_get_argument_uint32(arguments, 0);
  sli_zigbee_af_comms_hub_function_set_default_timeout(timeout);
}

static void sendMessage(sl_802154_long_addr_t deviceId, uint16_t length, uint8_t *message, bool includeHeader, uint16_t messageCode)
{
  uint8_t i;
  uint8_t *data = &(messagePayload[nextMessage][0]);
  uint16_t headerLength = 0;
  sl_zigbee_af_plugin_comms_hub_function_status_t status;

  // Tack on a dummy header
  if (includeHeader) {
    for (i = 0; i < SL_ZIGBEE_AF_PLUGIN_COMMS_HUB_FUNCTION_GBZ_OFFSET; i++) {
      *data++ = i;
    }
    headerLength = SL_ZIGBEE_AF_PLUGIN_COMMS_HUB_FUNCTION_GBZ_OFFSET;
  }

  if (headerLength + length > SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_SEND_LENGTH) {
    sl_zigbee_af_comms_hub_function_println("Message is too long.  Truncating excess bytes.");
    length = SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_SEND_LENGTH - headerLength;
  }
  memcpy(data, message, length);

  status = sl_zigbee_af_comms_hub_function_send(deviceId, headerLength + length, messagePayload[nextMessage], messageCode);
  if (status == SL_ZIGBEE_AF_CHF_STATUS_SUCCESS) {
    sl_zigbee_af_comms_hub_function_println("Message has been successfully sent or queued to be sent to the destination");
  }
  if (++nextMessage == SL_ZIGBEE_AF_COMMS_HUB_FUNCTION_MSG_CACHE) {
    nextMessage = 0;
  }
}

// plugin comms-hub-function get-tunnel-endpoint <uint16_t:nodeId>
void sli_zigbee_af_comms_hub_function_cli_get_tunnel_endpoint(sl_cli_command_arg_t *arguments)
{
  uint8_t  endpoint;
  uint16_t nodeId;

  nodeId = sl_cli_get_argument_uint16(arguments, 0);
  endpoint = sl_zigbee_af_get_device_tunneling_endpoint(nodeId);
  if ( endpoint == INVALID_TUNNELING_ENDPOINT ) {
    sl_zigbee_af_comms_hub_function_println("Invalid Tunneling Endpoint for 0x%2x", nodeId);
  } else {
    sl_zigbee_af_comms_hub_function_println("Tunnel Endpoint=0x%x", endpoint);
  }
}

void sli_zigbee_af_comms_hub_function_cli_close_tunnel(sl_cli_command_arg_t *arguments)
{
  sl_802154_long_addr_t deviceId;

  sl_zigbee_copy_eui64_arg(arguments, 0, deviceId, true);
  sli_zigbee_af_comms_hub_function_tunnel_close(deviceId);
}

void sli_zigbee_af_comms_hub_function_cli_create_tunnel(sl_cli_command_arg_t *arguments)
{
  sl_802154_long_addr_t deviceId;
  uint8_t endpoint;

  endpoint = sl_cli_get_argument_uint8(arguments, 1);

  sl_zigbee_copy_eui64_arg(arguments, 0, deviceId, true);
  sli_zigbee_af_comms_hub_function_tunnel_create(deviceId, endpoint);
}

void sli_zigbee_af_comms_hub_function_cli_print(sl_cli_command_arg_t *arguments)
{
  sli_zigbee_af_comms_hub_function_print();
}
