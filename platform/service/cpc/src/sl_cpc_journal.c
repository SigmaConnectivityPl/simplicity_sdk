/***************************************************************************/ /**
 * @file
 * @brief CPC API implementation.
 *******************************************************************************
 * # License
 * <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
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
#include "sl_status.h"
#include "sl_sleeptimer.h"
#include "sl_cpc_journal.h"
#include "sl_cpc_journal_config.h"
#include "sli_cpc.h"

typedef __PACKED_STRUCT {
  const char *str;
  uint32_t value;
  uint32_t timestamp;
  sl_cpc_journal_level_t level;
} sl_cpc_journal_object_t;

typedef struct {
  uint16_t head;
  uint16_t tail;
  uint32_t size;
  sl_cpc_journal_object_t buffer[SL_CPC_JOURNAL_MAX_ENTRY_COUNT];
} sl_cpc_journal_circular_buffer_t;

// Made volatile to ensure it does not get optimized out and is accessible by a debugger
volatile sl_cpc_journal_circular_buffer_t sl_cpc_journal_cb;

/***************************************************************************//**
 * Convert the journal levels to a string
 ******************************************************************************/
static const char* level_to_string(sl_cpc_journal_level_t level)
{
  switch (level) {
    case SL_CPC_JOURNAL_ERROR_LEVEL:   return "ERROR";
    case SL_CPC_JOURNAL_WARNING_LEVEL: return "WARNING";
    case SL_CPC_JOURNAL_INFO_LEVEL:    return "INFO";
    case SL_CPC_JOURNAL_DEBUG_LEVEL:   return "DEBUG";
    case SL_CPC_JOURNAL_TRACE_LEVEL:   return "TRACE";
    default:                           return "UNKNOWN";
  }
}

/***************************************************************************//**
 * Record an entry to the journal's circular buffer. If the provided journal level
 * is superior to the configured max level, the journal entry will be ignored.
 ******************************************************************************/
void sl_cpc_journal_record(sl_cpc_journal_level_t level, const char *string, uint32_t value)
{
  MCU_DECLARE_IRQ_STATE;

  // Drop if level if superior to configured level
  if (level > SL_CPC_JOURNAL_LEVEL) {
    return;
  }

  MCU_ENTER_ATOMIC();

  uint16_t next_head = (sl_cpc_journal_cb.head + 1) % SL_CPC_JOURNAL_MAX_ENTRY_COUNT;

  if (next_head == sl_cpc_journal_cb.tail) {
    // Buffer is full
    sl_cpc_journal_cb.tail = (sl_cpc_journal_cb.tail + 1) % SL_CPC_JOURNAL_MAX_ENTRY_COUNT;
  }

  sl_cpc_journal_cb.buffer[sl_cpc_journal_cb.head].str = string;
  sl_cpc_journal_cb.buffer[sl_cpc_journal_cb.head].value = value;
  sl_cpc_journal_cb.buffer[sl_cpc_journal_cb.head].level = level;
  sl_cpc_journal_cb.buffer[sl_cpc_journal_cb.head].timestamp = sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count());

  sl_cpc_journal_cb.head = next_head;

  MCU_EXIT_ATOMIC();
}

sl_status_t sl_cpc_journal_pop(sl_cpc_journal_object_t *object)
{
  MCU_DECLARE_IRQ_STATE;
  if (object == NULL) {
    return SL_STATUS_NULL_POINTER;
  }

  MCU_ENTER_ATOMIC();

  // Since sl_cpc_journal_cb is volatile, access order needs to be defined
  uint16_t head_temp = sl_cpc_journal_cb.head;
  uint16_t tail_temp = sl_cpc_journal_cb.tail;

  if (head_temp == tail_temp) {
    MCU_EXIT_ATOMIC();
    return SL_STATUS_EMPTY;
  }

  *object = sl_cpc_journal_cb.buffer[sl_cpc_journal_cb.tail];
  sl_cpc_journal_cb.tail = (sl_cpc_journal_cb.tail + 1) % SL_CPC_JOURNAL_MAX_ENTRY_COUNT;

  MCU_EXIT_ATOMIC();

  return SL_STATUS_OK;
}

/***************************************************************************//**
 * Initialize CPC journal module.
 ******************************************************************************/
void sl_cpc_journal_init(void)
{
  sl_cpc_journal_cb.head = 0;
  sl_cpc_journal_cb.tail = 0;
  sl_cpc_journal_cb.size = SL_CPC_JOURNAL_MAX_ENTRY_COUNT;
  SL_CPC_JOURNAL_RECORD_INFO("CPC journal Init", 0);
}

#if defined(SL_CATALOG_IOSTREAM_PRESENT)
/***************************************************************************//**
 * Print the contents of the CPC journal, this will consume the entries
 ******************************************************************************/
void sl_cpc_journal_print(bool print_csv_header, sl_iostream_t *stream)
{
  sl_cpc_journal_object_t current_object;
  sl_status_t status;

  if (print_csv_header) {
    sl_iostream_printf(stream, "Timestamp,Level,Message,Value(Hex)\n");
  }

  status = sl_cpc_journal_pop(&current_object);
  while (status != SL_STATUS_EMPTY) {
    sl_iostream_printf(stream, "%lu,%s,%s,0x%lx\n",
                       (unsigned long)current_object.timestamp,
                       level_to_string(current_object.level),
                       current_object.str,
                       (unsigned long)current_object.value);

    status = sl_cpc_journal_pop(&current_object);
  }
}
#endif

#if defined(SL_CATALOG_CPC_JOURNAL_CLI_PRESENT)
/***************************************************************************//**
 * CLI Command to print the contents of the journal
 ******************************************************************************/
void sl_cpc_journal_print_cmd_handler(sl_cli_command_arg_t *arguments)
{
  sl_cpc_journal_print(true, arguments->handle->iostream_handle);
}
#endif
