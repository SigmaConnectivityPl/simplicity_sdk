/***************************************************************************/ /**
 * @file
 * @brief CPC Journal
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

#ifndef SL_CPC_JOURNAL_H
#define SL_CPC_JOURNAL_H

/***************************************************************************/ /**
 * @addtogroup cpc CPC Journal
 * @{
 ******************************************************************************/
#include "sl_enum.h"

#if defined(SL_COMPONENT_CATALOG_PRESENT)
#include "sl_component_catalog.h"
#endif

#if defined(SL_CATALOG_CPC_JOURNAL_CLI_PRESENT)
#include "sl_cli.h"
#endif

#if defined(SL_CATALOG_IOSTREAM_PRESENT)
#include "sl_iostream.h"
#endif

/**
 * @brief Record a CPC journal entry.
 *
 * @param[in] string The message string to record.
 * @param[in] value The value associated with the message.
 */
#define SL_CPC_JOURNAL_RECORD_ERROR(string, value) \
  sl_cpc_journal_record(SL_CPC_JOURNAL_ERROR_LEVEL, string, value)
#define SL_CPC_JOURNAL_RECORD_WARNING(string, value) \
  sl_cpc_journal_record(SL_CPC_JOURNAL_WARNING_LEVEL, string, value)
#define SL_CPC_JOURNAL_RECORD_INFO(string, value) \
  sl_cpc_journal_record(SL_CPC_JOURNAL_INFO_LEVEL, string, value)
#define SL_CPC_JOURNAL_RECORD_DEBUG(string, value) \
  sl_cpc_journal_record(SL_CPC_JOURNAL_DEBUG_LEVEL, string, value)
#define SL_CPC_JOURNAL_RECORD_TRACE(string, value) \
  sl_cpc_journal_record(SL_CPC_JOURNAL_TRACE_LEVEL, string, value)

/// @brief Enumeration of journal entry levels.
SL_ENUM_GENERIC(sl_cpc_journal_level_t, uint8_t){
  SL_CPC_JOURNAL_ERROR_LEVEL = 1,
  SL_CPC_JOURNAL_WARNING_LEVEL = 2,
  SL_CPC_JOURNAL_INFO_LEVEL = 3,
  SL_CPC_JOURNAL_DEBUG_LEVEL = 4,
  SL_CPC_JOURNAL_TRACE_LEVEL = 5,
};

/**
 * @brief Initialize the CPC journal module.
 */
void sl_cpc_journal_init(void);

#if defined(SL_CATALOG_IOSTREAM_PRESENT) || defined(DOXYGEN)
/**
 * @brief Print the contents of the CPC journal via the default IOStream.
 *
 * @param[in] print_header Whether to print a CSV header.
 *
 * @param[in] stream The IOStream used to print the journal entries.
 *
 * @note Journal entries will be consumed when called
 */
void sl_cpc_journal_print(bool print_csv_header, sl_iostream_t *stream);
#endif

/**
 * @brief Record a journal entry.
 *
 * @param[in] level The severity level of the entry.
 * @param[in] string The message string to log.
 * @param[in] value The value associated with the message.
 */
void sl_cpc_journal_record(sl_cpc_journal_level_t level, const char *string, uint32_t value);

#if defined(SL_CATALOG_CPC_JOURNAL_CLI_PRESENT) || defined(DOXYGEN)
/**
 * @brief Command handler to print the contents of the journal.
 *
 * @param[in] arguments CLI arguments.
 *
 * @note Journal entries will be consumed when called
 */
void sl_cpc_journal_print_cmd_handler(sl_cli_command_arg_t *arguments);
#endif

#endif // SL_CPC_JOURNAL_H
