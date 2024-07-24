/***************************************************************************//**
 * @file
 * @brief CS Initiator display core logic
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
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "glib.h"
#include "dmd/dmd.h"

#include "sl_rtl_clib_api.h"
#include "cs_initiator_display_config.h"
#include "cs_initiator_display_core.h"
#include "cs_initiator_display.h"

// -----------------------------------------------------------------------------
// Macros

#define DEFAULT_FONT_TYPE                        ((GLIB_Font_t *)&GLIB_FontNarrow6x8)
#define STRING_LEN                               40

// -----------------------------------------------------------------------------
// Static function declarations
static void clear_row(uint8_t row);

// -----------------------------------------------------------------------------
// Static variables
static GLIB_Context_t glib_context;
static GLIB_Align_t glib_text_alignment;
static bool scan_on = false;

cs_initiator_display_content_t lcd_content;
cs_initiator_display_content_t prev_lcd_content;

/******************************************************************************
 * CS Initiator display clear specified row.
 *****************************************************************************/
static void clear_row(uint8_t row)
{
  const char empty[] = "                    ";
  GLIB_drawStringOnLine(&glib_context,
                        empty,
                        row,
                        GLIB_ALIGN_LEFT,
                        0,
                        0,
                        true);
}

/******************************************************************************
 * CS Initiator display init.
 *****************************************************************************/
sl_status_t cs_initiator_display_init(void)
{
  EMSTATUS status;

  memset(&lcd_content, 0, sizeof(cs_initiator_display_content_t));
  memset(&prev_lcd_content, 0, sizeof(cs_initiator_display_content_t));

  status = DMD_init(0);
  if (status != DMD_OK) {
    display_log_error("DMD_init failed! [E:0x%x]" NL, status);
    return SL_STATUS_INITIALIZATION;
  }

  status = GLIB_contextInit(&glib_context);
  if (status != GLIB_OK) {
    display_log_error("DMD_init failed! [E:0x%x]" NL, status);
    return SL_STATUS_INITIALIZATION;
  }

  glib_context.backgroundColor = White;
  glib_context.foregroundColor = Black;

  GLIB_setFont(&glib_context, DEFAULT_FONT_TYPE);
  GLIB_clear(&glib_context);
  cs_initiator_display_set_alignment(CS_INITIATOR_DISPLAY_ALIGNMENT_CENTER);

  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_TITLE_VENDOR_TEXT, ROW_SYSTEM);
  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_TITLE_DEVICE_TEXT, ROW_ROLE);

  cs_initiator_display_set_alignment(CS_INITIATOR_DISPLAY_ALIGNMENT_LEFT);

  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_DISTANCE_TEXT,
                                  ROW_DISTANCE_TEXT);

  cs_initiator_display_print_float_value(lcd_content.distance,
                                         ROW_DISTANCE_VALUE,
                                         "m");

  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_RSSI_DISTANCE_TEXT, ROW_RSSI_DISTANCE_TEXT);
  cs_initiator_display_print_float_value(lcd_content.rssi_distance, ROW_RSSI_DISTANCE_VALUE, "m");
  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_LIKELINESS_TEXT, ROW_LIKELINESS_TEXT);
  cs_initiator_display_print_float_value(lcd_content.likeliness, ROW_LIKELINESS_VALUE, NULL);

  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_BER_TEXT, ROW_BIT_ERROR_RATE_TEXT);
  cs_initiator_display_print_float_value(lcd_content.bit_error_rate, ROW_BIT_ERROR_RATE_VALUE, NULL);

  cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_STATE_SCANNING_TEXT, ROW_STATE);

  // force update LCD content for drawing the first screen
  cs_initiator_display_update();
  return SL_STATUS_OK;
}

/******************************************************************************
 * CS Initiator display update display content.
 *****************************************************************************/
void cs_initiator_display_update(void)
{
  DMD_updateDisplay();
}

/******************************************************************************
 * CS Initiator display set display alignment.
 *****************************************************************************/
void cs_initiator_display_set_alignment(cs_initiator_display_alignment_t alignment)
{
  switch (alignment) {
    case CS_INITIATOR_DISPLAY_ALIGNMENT_LEFT:
      glib_text_alignment = GLIB_ALIGN_LEFT;
      break;

    case CS_INITIATOR_DISPLAY_ALIGNMENT_CENTER:
      glib_text_alignment = GLIB_ALIGN_CENTER;
      break;

    case CS_INITIATOR_DISPLAY_ALIGNMENT_RIGHT:
      glib_text_alignment = GLIB_ALIGN_RIGHT;
      break;
  }
}

/******************************************************************************
 * CS Initiator display write string to a specified row.
 *****************************************************************************/
void cs_initiator_display_write_text(char *str, uint8_t row)
{
  if (!strlen(str)) {
    return;
  }
  clear_row(row);
  GLIB_drawStringOnLine(&glib_context,
                        str,
                        row,
                        glib_text_alignment,
                        0,
                        0,
                        true);
}

// -----------------------------------------------------------------------------
// Event / callback declarations

/******************************************************************************
 * Bluetooth stack event handler.
 *****************************************************************************/
void cs_initiator_display_on_event(sl_bt_msg_t *evt)
{
  bool update_display = false;

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_STATE_INITIALIZED_TEXT, ROW_STATE);
      update_display = true;
      break;
    case sl_bt_evt_connection_opened_id:
      cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_STATE_CONNECTED_TEXT, ROW_STATE);
      scan_on = false;
      update_display = true;
      break;
    case sl_bt_evt_connection_closed_id:
      cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_STATE_DISCONNECTED_TEXT, ROW_STATE);
      update_display = true;
      break;
    // do the same for these 2 events
    case sl_bt_evt_scanner_extended_advertisement_report_id:
    case sl_bt_evt_scanner_legacy_advertisement_report_id:
      if (!scan_on) {
        scan_on = true;
        cs_initiator_display_write_text(CS_INITIATOR_DISPLAY_STATE_SCANNING_TEXT, ROW_STATE);
        update_display = true;
      }
      break;
    default:
      break;
  }

  // update display only in case its necessary
  if (update_display) {
    cs_initiator_display_update();
  }
}
