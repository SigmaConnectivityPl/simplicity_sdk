/**
 * @file
 * @brief Weak implementation for Command Class User Credential manufacturer
 * Credential validation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2024 Silicon Laboratories Inc.
 *
 * @copyright 2024 Silicon Laboratories Inc.
 */

#include "CC_UserCredential.h"

/**
 * @brief Validate the PIN Code credential
 *
 * @param[in] p_credential Pointer to the credential to validate
 *
 * @return true if the credential is valid, false otherwise
 */
static bool CC_UserCredential_manufacturer_validate_pin_code(u3c_credential * p_credential)
{
  // The following requirements are marked as SHOULD in the specification.
  // Manufacturers can change freely.

  // Check if the PIN Code only contains the same digit
  // CC:0083.01.00.42.015
  bool is_same_digit = true;
  for (uint8_t i = 1; i < p_credential->metadata.length; ++i) {
    if (p_credential->data[i] != p_credential->data[0]) {
      is_same_digit = false;
      break;
    }
  }

  if (is_same_digit) {
    return false;
  }

  // Check if the PIN Code digits are ascending or descending
  // CC:0083.01.00.42.014
  bool is_ascending = true;
  bool is_descending = true;
  for (uint8_t i = 1; i < p_credential->metadata.length; ++i) {
    if (p_credential->data[i] != (p_credential->data[i - 1] + 1)) {
      is_ascending = false;
    }
    if (p_credential->data[i] != (p_credential->data[i - 1] - 1)) {
      is_descending = false;
    }
  }

  if (is_ascending || is_descending) {
    return false;
  }

  return true;
}

ZW_WEAK bool CC_UserCredential_manufacturer_validate_credential(
  u3c_credential * credential)
{
  bool result = true;
  switch (credential->metadata.type) {
    case CREDENTIAL_TYPE_PIN_CODE:
      result = CC_UserCredential_manufacturer_validate_pin_code(credential);
      break;
    default:
      break;
  }
  return result;
}

ZW_WEAK bool CC_UserCredential_manufacturer_validate_admin_pin_code(
  __attribute__((unused))u3c_admin_code_metadata_t * code
  )
{
  code->result = ADMIN_CODE_OPERATION_RESULT_INTERNAL_NONE; 
  return true; 
}
