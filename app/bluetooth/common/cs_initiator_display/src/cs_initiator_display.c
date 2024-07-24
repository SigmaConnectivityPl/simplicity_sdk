/***************************************************************************//**
 * @file
 * @brief CS Initiator display
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

#include "sl_rtl_clib_api.h"

#include "cs_initiator_display_config.h"
#include "cs_initiator_display_core.h"
#include "cs_initiator_display.h"

// -----------------------------------------------------------------------------
// Macros

#define FONT_TYPE                                ((GLIB_Font_t *)&GLIB_FontNarrow6x8)
#define STRING_LEN                               40

extern cs_initiator_display_content_t lcd_content, prev_lcd_content;

static sl_status_t cs_initiator_display_measurement_modes(sl_bt_cs_mode_t mode,
                                                          uint8_t algo_mode,
                                                          uint8_t row);

static void cs_initiator_display_distance_measurement(float value,
                                                      uint8_t percentage,
                                                      uint8_t row);

// -----------------------------------------------------------------------------
// Public function definitions

/******************************************************************************
 * Set distance value to display
 *****************************************************************************/
void cs_initiator_display_set_distance(float distance)
{
  if (distance != lcd_content.distance) {
    lcd_content.distance = distance;
  }
  cs_initiator_display_distance_measurement(lcd_content.distance,
                                            100u,
                                            ROW_DISTANCE_VALUE);
  cs_initiator_display_update();
}

/******************************************************************************
 * Set distance progress percentage to display
 *****************************************************************************/
void cs_initiator_display_set_distance_progress(float progress_percentage)
{
  if (progress_percentage != lcd_content.progress_percentage) {
    lcd_content.progress_percentage = progress_percentage;
  }
  cs_initiator_display_distance_measurement(lcd_content.distance,
                                            (uint8_t)lcd_content.progress_percentage,
                                            ROW_DISTANCE_VALUE);
  cs_initiator_display_update();
}

/******************************************************************************
 * Set RSSI based distance value to display
 *****************************************************************************/
void cs_initiator_display_set_rssi_distance(float distance)
{
  if (distance != lcd_content.rssi_distance) {
    lcd_content.rssi_distance = distance;
  }

  cs_initiator_display_distance_measurement(lcd_content.rssi_distance,
                                            100u,
                                            ROW_RSSI_DISTANCE_VALUE);

  cs_initiator_display_update();
}

/******************************************************************************
 * Set the likeliness parameter to display
 *****************************************************************************/
void cs_initiator_display_set_likeliness(float likeliness)
{
  if (likeliness != lcd_content.likeliness) {
    lcd_content.likeliness = likeliness;
  }

  cs_initiator_display_print_float_value(lcd_content.likeliness,
                                         ROW_LIKELINESS_VALUE,
                                         NULL);

  cs_initiator_display_update();
}

/******************************************************************************
 * Set the Bit Error Rate (BER) value to display
 *****************************************************************************/
void cs_initiator_display_set_bit_error_rate(float ber)
{
  if (ber != lcd_content.bit_error_rate) {
    lcd_content.bit_error_rate = ber;
  }

  cs_initiator_display_print_float_value(lcd_content.bit_error_rate,
                                         ROW_BIT_ERROR_RATE_VALUE,
                                         NULL);
  cs_initiator_display_update();
}

/******************************************************************************
 * CS Initiator display print value with a specified unit.
 *****************************************************************************/
void cs_initiator_display_print_float_value(float value, uint8_t row, char *unit)
{
  char *unit_str = "";
  if (unit != NULL) {
    unit_str = unit;
  }
  char buffer[STRING_LEN];
  uint32_t base = truncf(value);
  uint32_t ext = (value - (float)base) * 100;
  sprintf(buffer, "%02lu.%02lu %s", base, ext, unit_str);
  cs_initiator_display_write_text(buffer, row);
}

/******************************************************************************
 * Set the measurement mode and object tracking mode and show on LCD
 *****************************************************************************/
void cs_initiator_display_set_measurement_mode(sl_bt_cs_mode_t mode,
                                               uint8_t algo_mode)
{
  sl_status_t sc = SL_STATUS_OK;

  if (mode != lcd_content.mode) {
    lcd_content.mode = mode;
  }
  if (algo_mode != lcd_content.algo_mode) {
    lcd_content.algo_mode = algo_mode;
  }
  sc = cs_initiator_display_measurement_modes(mode, algo_mode, ROW_MODE);
  if (sc != SL_STATUS_OK) {
    display_log_error("Error during showing measurement mode and "
                      "algo mode on LCD! [E: 0x%x]" NL, sc);
  } else {
    cs_initiator_display_update();
  }
}

// -----------------------------------------------------------------------------
// Private function definitions

/******************************************************************************
 * CS initiator display measurement mode and object tracking mode.
 *****************************************************************************/
static sl_status_t cs_initiator_display_measurement_modes(sl_bt_cs_mode_t mode,
                                                          uint8_t algo_mode,
                                                          uint8_t row)
{
  sl_status_t sc = SL_STATUS_OK;
  char string[STRING_LEN] = "\0";

  if (mode == sl_bt_cs_mode_rtt) {
    if (strncat(string, CS_INITIATOR_DISPLAY_MODE_RTT_TEXT, (sizeof(string) - strlen(string) - 1u)) == NULL) {
      display_log_error("Failed to concat \'%s\' string!" NL,
                        CS_INITIATOR_DISPLAY_MODE_RTT_TEXT);
      sc = SL_STATUS_FAIL;
    }
  } else {
    if (strncat(string, CS_INITIATOR_DISPLAY_MODE_PBR_TEXT, (sizeof(string) - strlen(string) - 1u)) == NULL) {
      display_log_error("Failed to concat \'%s\' string!" NL,
                        CS_INITIATOR_DISPLAY_MODE_PBR_TEXT);
      sc = SL_STATUS_FAIL;
    }
  }

  if (algo_mode == SL_RTL_CS_ALGO_MODE_REAL_TIME_BASIC) {
    if (strncat(string, CS_INITIATOR_DISPLAY_AMODE_MOVING_OBJ_TEXT, (sizeof(string) - strlen(string) - 1u)) == NULL) {
      display_log_error("Failed to concat \'%s\' string!" NL,
                        CS_INITIATOR_DISPLAY_ALGO_MODE_MOVING_OBJ_TEXT);
      sc = SL_STATUS_FAIL;
    }
  } else {
    if (strncat(string, CS_INITIATOR_DISPLAY_AMODE_STATIONARY_OBJ_TEXT, (sizeof(string) - strlen(string) - 1u)) == NULL) {
      display_log_error("Failed to concat \'%s\' string!" NL,
                        CS_INITIATOR_DISPLAY_ALGO_MODE_STATIONARY_OBJ_TEXT);
      sc = SL_STATUS_FAIL;
    }
  }

  cs_initiator_display_write_text(string, row);
  return sc;
}

/******************************************************************************
 * CS initiator display distance measurement percentage in case of measuring is
 * still in progress. Otherwise display measured distance only.
 *****************************************************************************/
static void cs_initiator_display_distance_measurement(float value,
                                                      uint8_t percentage,
                                                      uint8_t row)
{
  char buffer[STRING_LEN];

  if (percentage == 100) {
    cs_initiator_display_print_float_value(value, row, "m");
  } else {
    uint32_t base = truncf(value);
    uint32_t ext = (value - (float)base) * 100;
    sprintf(buffer, "%02lu.%02lu m (%02u%%)",
            (unsigned long)base,
            (unsigned long)ext,
            (unsigned int)percentage);
    cs_initiator_display_write_text(buffer, row);
  }
}
