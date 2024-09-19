/**
 * @file
 * User Credential Command Class credential validation.
 *
 * @copyright 2024 Silicon Laboratories Inc.
 */

#ifndef CC_USER_CREDENTIAL_VALIDATION_H
#define CC_USER_CREDENTIAL_VALIDATION_H

#include "CC_UserCredential.h"

/**
 * Function pointer type for Credential Type Validators
 */
typedef bool (*u3c_credential_type_validator_t)(u3c_credential*, RECEIVE_OPTIONS_TYPE_EX*);

/**
 * Function for validating a Credential
 */
bool validate_credential_data(u3c_credential * p_credential, RECEIVE_OPTIONS_TYPE_EX * p_rx_options);

/**
 * @}
 * @}
 */

#endif /* CC_USER_CREDENTIAL_VALIDATION_H */
