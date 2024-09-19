/**
 * @file
 * @brief User Credential data validation functions
 * @details Contains functions for validating User Credential data of each
 * credential type against the rules set out by the specification.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2024 Silicon Laboratories Inc.
 */

#include "cc_user_credential_validation.h"
#include "cc_user_credential_io.h"
#include "cc_user_credential_config_api.h"
#include "assert.h"
#include <string.h>

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

static bool is_identical_to_admin_pin_code(
  u3c_credential * p_credential, RECEIVE_OPTIONS_TYPE_EX * p_rx_options
  )
{
  if (!cc_user_credential_get_admin_code_supported()) {
    return false;
  }

  // Read the admin code from the database
  u3c_admin_code_metadata_t admin_code_metadata = { 0 };
  u3c_db_operation_result get_admin_code_result =
    CC_UserCredential_get_admin_code_info(&admin_code_metadata);
  if (get_admin_code_result != U3C_DB_OPERATION_RESULT_SUCCESS) {
    // Could not read the admin code
    assert(false);
    return false;
  }

  if (admin_code_metadata.code_length != p_credential->metadata.length) {
    /**
     * The provided PIN code cannot be identical to the admin PIN code if its
     * length is different
     */
    return false;
  }

  int8_t comparison_result = strncmp(
    (char *)p_credential->data, (char *)admin_code_metadata.code_data,
    admin_code_metadata.code_length);

  if (comparison_result == 0) {
    // The PIN codes are identical, report duplicate of admin PIN code

    // Get next credential's details
    u3c_credential_type next_credential_type = CREDENTIAL_TYPE_NONE;
    uint16_t next_credential_slot = 0;
    CC_UserCredential_get_next_credential(
      0, p_credential->metadata.type, p_credential->metadata.slot,
      &next_credential_type, &next_credential_slot
      );

    // Send report
    CC_UserCredential_CredentialReport_tx(
      CREDENTIAL_REP_TYPE_DUPLICATE_ADMIN_PIN_CODE, p_credential,
      next_credential_type, next_credential_slot, p_rx_options);
    return true;
  }

  return false;
}

/**
 * Validates a PIN Code Credential
 *
 * @return true if the Credential is valid
 */
static bool validate_pin_code(u3c_credential * p_credential, RECEIVE_OPTIONS_TYPE_EX * p_rx_options)
{
  // If the Admin Code is supported, the PIN Code must not match the Admin Code
  if (is_identical_to_admin_pin_code(p_credential, p_rx_options)) {
    return false;
  }

  // PIN Code must be at least 4 digits long
  // CC:0083.01.00.41.016
  if (p_credential->metadata.length < U3C_CREDENTIAL_TYPE_PIN_CODE_MIN_LENGTH_REQUIREMENT) {
    return false;
  }

  // Only allow numeric PIN Codes
  // CC:0083.01.0A.11.000
  for (uint8_t i = 0; i < p_credential->metadata.length; ++i) {
    uint8_t character = p_credential->data[i];
    if (character < '0' || character > '9') {
      return false;
    }
  }

  return true;
}

/****************************************************************************/
/*                             STATIC VARIABLES                             */
/****************************************************************************/

static u3c_credential_type_validator_t u3c_credential_validator_functions[CREDENTIAL_TYPE_NUMBER_OF_TYPES] = {
  [CREDENTIAL_TYPE_PIN_CODE] = validate_pin_code,
};

/****************************************************************************/
/*                             PUBLIC FUNCTIONS                             */
/****************************************************************************/

bool validate_credential_data(u3c_credential * p_credential, RECEIVE_OPTIONS_TYPE_EX * p_rx_options)
{
  if (u3c_credential_validator_functions[p_credential->metadata.type]) {
    return (
      u3c_credential_validator_functions[p_credential->metadata.type](p_credential, p_rx_options)
      );
  }

  return true;
}
