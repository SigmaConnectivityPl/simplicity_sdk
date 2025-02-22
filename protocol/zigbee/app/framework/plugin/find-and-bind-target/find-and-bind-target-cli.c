/***************************************************************************//**
 * @file
 * @brief CLI for the Find and Bind Target plugin.
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

#include "find-and-bind-target.h"

// -----------------------------------------------------------------------------
// CLI Command Definitions

#include "sl_cli.h"

void sl_zigbee_af_find_and_bind_target_start_command(sl_cli_command_arg_t *arguments)
{
  uint8_t endpoint = sl_cli_get_argument_uint8(arguments, 0);

  sl_zigbee_af_find_and_bind_target_start(endpoint);
}
