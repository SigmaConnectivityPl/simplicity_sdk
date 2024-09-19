/***************************************************************************//**
 * @file
 * @brief cli_door_lock_keypad.c
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
//                                   Includes
// -----------------------------------------------------------------------------
#include "sl_component_catalog.h"

#ifdef SL_CATALOG_ZW_CLI_COMMON_PRESENT

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "zaf_event_distributor_soc.h"
#include "CC_UserCode.h"
#include "sl_cli.h"
#include "app_log.h"
#include "app_cli.h"
#include "CC_UserCredential.h"
#include "cc_user_credential_io.h"
#include "cc_user_credential_io_config.h"
#include "CC_DoorLock.h"
#include "events.h"
#include "cc_user_credential_config_api.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
typedef enum _u3c_credential_attribute_t {
  CREDENTIAL_ATTRIBUTE_NONE,
  CREDENTIAL_ATTRIBUTE_CREDENTIAL,
  CREDENTIAL_ATTRIBUTE_SLOT,
  CREDENTIAL_ATTRIBUTE_UUID,
  CREDENTIAL_ATTRIBUTE_END
} u3c_credential_attribute;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
extern void delete_all_credentials_of_type(uint16_t uuid, u3c_credential_type filter_type);

static bool u3c_add_user(char* user_name);
static bool u3c_add_credential(u3c_credential_type type, uint16_t uuid, uint16_t slot, unsigned char* credential_data);
static bool u3c_modify_user_name(u3c_user *user, char* name);
static bool u3c_modify_user_type(u3c_user *user, char* type);
static bool u3c_modify_user_active(u3c_user *user, char* state);
static bool u3c_modify_credential(uint16_t slot, u3c_credential_type type, unsigned char* credential_data);
static bool u3c_move_credential_slot(uint16_t uuid, uint16_t slot, u3c_credential_type type, uint16_t new_slot);
static bool u3c_move_credential_uuid(uint16_t uuid, uint16_t slot, u3c_credential_type type, uint16_t new_uuid);
static u3c_credential_type u3c_convert_str_to_credential_type(unsigned char* str);
static u3c_user_type u3c_convert_str_to_user_type(char* str);
static u3c_credential_attribute u3c_convert_str_to_credential_attribute(char* str);
static uint16_t u3c_generate_uuid(void);
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/*
 * @brief CLI command to add a user.
 *
 * This function adds a user.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_add_user(sl_cli_command_arg_t *arguments)
{
  char* user_name = sl_cli_get_argument_string(arguments, 0);

  if (u3c_add_user(user_name) == true) {
    app_log_info("%s stored successfully!\r\n", user_name);
  } else {
    app_log_error("Storing %s failed!\r\n", user_name);
  }
}

/**
 * @brief Adds a credential to the database.
 *
 * This function is used to add a new credential to the database. It takes the
 * credential information as input parameters and stores it in the database.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_add_credential(sl_cli_command_arg_t *arguments)
{
  uint16_t uuid    = sl_cli_get_argument_uint16(arguments, 0);
  uint16_t slot    = sl_cli_get_argument_uint16(arguments, 1);
  char* type       = sl_cli_get_argument_string(arguments, 2);
  char* credential = sl_cli_get_argument_string(arguments, 3);
  u3c_credential_type credential_type;

  credential_type = u3c_convert_str_to_credential_type((unsigned char*)type);

  if ((credential_type != CREDENTIAL_TYPE_NONE) && (credential_type != CREDENTIAL_TYPE_NUMBER_OF_TYPES)) {
    app_log_info("Storing new %s credential.\r\n", type);
    if (u3c_add_credential(credential_type, uuid, slot, (unsigned char*)credential) == true) {
      app_log_info("%s stored successfully!\r\n", type);
    } else {
      app_log_error("Storing %s failed!\r\n", type);
    }
  }
}

/**
 * @brief Modifies a user in the database.
 *
 * This function is used to modify the details of a user in the database.
 * It takes the user ID and the new user details as parameters and updates
 * the user's information in the database.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_modify_user(sl_cli_command_arg_t *arguments)
{
  uint16_t uuid         = sl_cli_get_argument_uint16(arguments, 0);
  char* attribute       = sl_cli_get_argument_string(arguments, 1);
  char* attribute_value = sl_cli_get_argument_string(arguments, 2);
  u3c_user user;

  if (CC_UserCredential_get_user(uuid, &user, NULL) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    if (0 == strcmp(attribute, "type")) {
      if (u3c_modify_user_type(&user, attribute_value)) {
        app_log_info("User update with new type %s was sucessful\r\n", attribute_value);
      } else {
        app_log_error("User update failed with new type %s!\r\n", attribute_value);
      }
    } else if (0 == strcmp(attribute, "name")) {
      if (u3c_modify_user_name(&user, attribute_value)) {
        app_log_info("User update with new name %s was sucessful\r\n", attribute_value);
      } else {
        app_log_error("User update failed with name %s\r\n", attribute_value);
      }
    } else if (0 == strcmp(attribute, "active")) {
      if (u3c_modify_user_active(&user, attribute_value)) {
        app_log_info("User activation was successful\r\n");
      } else {
        app_log_error("User activation failed!\r\n");
      }
    } else {
      app_log_error("%s attribute is not supported!\r\n", attribute);
    }
  } else {
    app_log_error("User with uuid %d not found!\r\n", uuid);
  }
}

/**
 * @brief Modifies the credential for a given user in the database.
 *
 * This function allows the modification of the credential (e.g., password) for a specific user in the database.
 * The function takes the slot and type of the credential to identify the credential in the database to modify.
 * It also takes an attribute name and its value to update. Plus the UID of the assigned user.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_modify_credential(sl_cli_command_arg_t *arguments)
{
  bool update_status = false;
  uint16_t uuid = sl_cli_get_argument_uint16(arguments, 0);
  uint16_t slot = sl_cli_get_argument_uint16(arguments, 1);
  char* type    = sl_cli_get_argument_string(arguments, 2);
  char* attribute       = sl_cli_get_argument_string(arguments, 3);
  char* attribute_value = sl_cli_get_argument_string(arguments, 4);

  u3c_credential_type credential_type = u3c_convert_str_to_credential_type((unsigned char*)type);
  u3c_credential_attribute attribute_to_change = u3c_convert_str_to_credential_attribute(attribute);

  if (credential_type != CREDENTIAL_TYPE_NUMBER_OF_TYPES) {
    switch (attribute_to_change) {
      case CREDENTIAL_ATTRIBUTE_CREDENTIAL:
        update_status = u3c_modify_credential(slot, credential_type, (unsigned char*)attribute_value);
        break;
      case CREDENTIAL_ATTRIBUTE_SLOT:
        update_status = u3c_move_credential_slot(uuid, slot, credential_type, atoi(attribute_value));
        break;
      case CREDENTIAL_ATTRIBUTE_UUID:
        update_status = u3c_move_credential_uuid(uuid, slot, credential_type, atoi(attribute_value));
        break;
      default:
        break;
    }
  }

  if (update_status) {
    app_log_info("Credential update was successful!\r\n");
  } else {
    app_log_error("Credential update failed!\r\n");
  }
}

/**
 * @brief Deletes a user from the database.
 *
 * This function deletes a user from the database based on the provided user ID.
 * The command also deletes all of the assigned credentials too.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_delete_user(sl_cli_command_arg_t *arguments)
{
  uint16_t uuid = sl_cli_get_argument_uint16(arguments, 0);

  if (CC_UserCredential_delete_user(uuid) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    app_log_info("User successfully deleted!\r\n");
    app_log_info("Deleting assiciated credentials\r\n");
    delete_all_credentials_of_type(uuid, CREDENTIAL_TYPE_NONE);
    app_log_info("Done\r\n");
  } else {
    app_log_error("User deletion failed!\r\n");
  }
}

/**
 * @brief Deletes a credential from the database.
 *
 * This function deletes a credential from the database based on the provided credential ID.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_db_delete_credential(sl_cli_command_arg_t *arguments)
{
  uint16_t uuid    = sl_cli_get_argument_uint16(arguments, 0);
  uint16_t slot    = sl_cli_get_argument_uint16(arguments, 1);
  char* type       = sl_cli_get_argument_string(arguments, 2);
  u3c_credential_type credential_type = u3c_convert_str_to_credential_type((unsigned char*)type);

  if ( credential_type != CREDENTIAL_TYPE_NUMBER_OF_TYPES) {
    if (CC_UserCredential_delete_credential(uuid, credential_type, slot) == U3C_DB_OPERATION_RESULT_SUCCESS) {
      app_log_info("Credential successfully deleted!\r\n");
    } else {
      app_log_error("Credential deletion failed!\r\n");
    }
  }
}

/**
 * @brief Enters a credential via the CLI.
 *
 * This function allows the user to enter a credential via the Command Line Interface (CLI).
 * The entered credential is then used for authentication and authorization purposes.
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_enter_credential(sl_cli_command_arg_t *arguments)
{
  static char credential_buffer[U3C_BUFFER_SIZE_CREDENTIAL_DATA] = { 0 };
  uint16_t uuid    = sl_cli_get_argument_uint16(arguments, 0);
  uint16_t slot    = sl_cli_get_argument_uint16(arguments, 1);
  char* type       = sl_cli_get_argument_string(arguments, 2);
  char* credential_data = sl_cli_get_argument_string(arguments, 3);
  uint16_t credential_size = 0;
  u3c_credential_type credential_type;

  credential_size = strlen(credential_data);

  memset(credential_buffer, 0, sizeof(credential_buffer));
  memcpy(credential_buffer, credential_data, credential_size);

  credential_type = u3c_convert_str_to_credential_type((unsigned char*)type);

  static u3c_credential credential = {
    .metadata = {
      .uuid = 1,
      .slot = 1,
      .modifier_node_id = 0,
      .modifier_type = MODIFIER_TYPE_LOCALLY
    }
  };
  static u3c_event_data_validate user_credential_event_validate_data = {
    .credential = &credential,
    .is_unlocked = true
  };

  if (credential_type != CREDENTIAL_TYPE_NUMBER_OF_TYPES) {
    credential.metadata.uuid = uuid;
    credential.metadata.slot = slot;
    credential.metadata.type = credential_type;
    credential.metadata.length = credential_size;
    credential.data = (uint8_t *) credential_buffer;

    user_credential_event_validate_data.is_unlocked = door_lock_hw_bolt_is_unlocked();
    uint16_t command_class = COMMAND_CLASS_USER_CREDENTIAL;
    uint8_t  cc_event =  CC_USER_CREDENTIAL_EVENT_VALIDATE;
    void* cc_data = &user_credential_event_validate_data;
    zaf_event_distributor_enqueue_cc_event(command_class, cc_event, cc_data);
  }
}

/**
 * @brief Sets the learn mode via the CLI.
 *
 * This function is used to set the learn mode via the Command Line Interface (CLI).
 * The command triggers the EVENT_APP_CREDENTIAL_LEARN_START event,
 *
 * @param arguments The arguments of the command.
 */
void cli_u3c_set_learn_mode(sl_cli_command_arg_t *arguments)
{
  uint16_t uuid    = sl_cli_get_argument_uint16(arguments, 0);
  uint16_t slot    = sl_cli_get_argument_uint16(arguments, 1);
  char* type       = sl_cli_get_argument_string(arguments, 2);

  uint16_t command_class = COMMAND_CLASS_NO_OPERATION;
  uint8_t cc_event = 0;
  void *cc_data = NULL;
  static u3c_credential_learn_event_data credential_learn_start_data;

  u3c_credential_type credential_type = u3c_convert_str_to_credential_type((unsigned char*)type);

  if (credential_type != CREDENTIAL_TYPE_NUMBER_OF_TYPES) {
    credential_learn_start_data.target.uuid = uuid;
    credential_learn_start_data.target.slot = slot;
    credential_learn_start_data.target.type = credential_type;
    credential_learn_start_data.operation_type = U3C_OPERATION_TYPE_ADD;
    credential_learn_start_data.source_id = 0;
    credential_learn_start_data.timeout_seconds = 0;

    // Simulates initiating the Credential Learn process locally
    command_class = COMMAND_CLASS_USER_CREDENTIAL;
    cc_event = CC_USER_CREDENTIAL_EVENT_LEARN_START_REQUEST;
    cc_data = &credential_learn_start_data;
    zaf_event_distributor_enqueue_cc_event(command_class, cc_event, cc_data);
  }
}

/**
 * @brief Adds a new user to the system.
 *
 * This function adds a new user to the system with the provided username and password.
 * It performs necessary validations and checks before adding the user.
 *
 * @param username The username of the user to be added.
 * @return Returns true if the user is successfully added, otherwise false.
 */
static bool u3c_add_user(char* user_name)
{
  static uint16_t uuid = 0;
  bool operation_result = false;

  uuid = u3c_generate_uuid();

  u3c_user user = {
    .active = true,
    .unique_identifier = uuid,
    .modifier_node_id = 0,
    .expiring_timeout_minutes = 0,
    .name_length = strlen(user_name),
    .type = USER_TYPE_GENERAL,
    .modifier_type = MODIFIER_TYPE_LOCALLY,
    .credential_rule = CREDENTIAL_RULE_SINGLE,
    .name_encoding = USER_NAME_ENCODING_STANDARD_ASCII,
  };

  if (CC_UserCredential_add_user(&user, (unsigned char*)user_name) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    operation_result = true;
    uuid--;
    app_log_info("User added with the following parameters:\r\n");
    app_log_append_info("\tname: %s\r\n", user_name);
    app_log_append_info("\tactive: %d\r\n", user.active);
    app_log_append_info("\tunique_identifier: %d\r\n", user.unique_identifier);
    app_log_append_info("\tmodifier_node_id: %d\r\n", user.modifier_node_id);
    app_log_append_info("\texpiring_timeout_minutes: %d\r\n", user.expiring_timeout_minutes);
    app_log_append_info("\tname_length: %d\r\n", user.name_length);
    app_log_append_info("\ttype: %d\r\n", user.type);
    app_log_append_info("\tmodifier_type: %d\r\n", user.modifier_type);
    app_log_append_info("\tcredential_rule: %d\r\n", user.credential_rule);
    app_log_append_info("\tname_encodings: %d\r\n", user.name_encoding);
  } else {
    app_log_error("Add user failed!\r\n");
  }
  return operation_result;
}

/**
 * @brief Modifies a credential.
 *
 * This function allows the modification of the credential data.
 * It takes the slot, type pair to identify the credential and the new credential data.
 *
 * @param slot Slot of the existing credential
 * @param type Type of the existing credential
 * @param credential_data New credential data
 */
static bool u3c_modify_credential(uint16_t slot, u3c_credential_type type, unsigned char* credential_data)
{
  u3c_credential credential = {
    .metadata = {
      .uuid = 0,
      .type = 0,
      .slot = 0,
      .modifier_node_id = 0,
      .length = 0,
      .modifier_type = 0
    },
    .data = NULL
  };
  bool operation_result = false;

  if (CC_UserCredential_get_credential(0, type, slot, &credential.metadata, credential.data) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    credential.metadata.modifier_type    = MODIFIER_TYPE_LOCALLY;
    credential.metadata.modifier_node_id = 0;
    credential.metadata.length           = sizeof(credential_data);
    credential.data                      = credential_data;

    if (CC_UserCredential_modify_credential(&credential) == U3C_DB_OPERATION_RESULT_SUCCESS) {
      operation_result = true;
    }
  }

  return operation_result;
}

/**
 * @brief Moves the credential to an another slot.
 *
 * The function moves an existing credential to a new slot.
 *
 * @param uuid UID of the assigned user
 * @param slot Current slot of the credential
 * @param type Type of the credential
 * @param new_slot Number of the new slot to move the credential to
 *
 * @return True if the move was successful, false otherwise.
 */
static bool u3c_move_credential_slot(uint16_t uuid, uint16_t slot, u3c_credential_type type, uint16_t new_slot)
{
  bool operation_result = false;

  if (CC_UserCredential_move_credential(uuid, type, slot, uuid, new_slot) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    operation_result = true;
  }

  return operation_result;
}

/**
 * @brief Assigns an existing credential to an another user.
 *
 * The function assigns an existing credential to an another existing user in the database
 *
 * @param uuid Current UID of the assigned user
 * @param slot Slot of the credential
 * @param type Type of the credential
 * @param new_uuid UID of the user to assign to credential to
 *
 * @return True if the move was successful, false otherwise.
 */
static bool u3c_move_credential_uuid(uint16_t uuid, uint16_t slot, u3c_credential_type type, uint16_t new_uuid)
{
  bool operation_result = false;

  if (CC_UserCredential_move_credential(uuid, type, slot, new_uuid, slot) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    operation_result = true;
  }

  return operation_result;
}

/**
 * @brief Adds a new credential to the database.
 *
 * This function allows the user to add a new credential to the database.
 *
 * @param type u3c_credential_type instance to set the type of the credential.
 * @param uuid Assigned user's uid.
 * @param slot Number of the slot where to store the new credential.
 * @param credential_data Actual data of the credential.
 *
 * @return Returns true if the credential was successfully added, or a false
 *         error code if an error occurred.
 */
static bool u3c_add_credential(u3c_credential_type type, uint16_t uuid, uint16_t slot, unsigned char* credential_data)
{
  bool operation_result = false;

  u3c_credential credential = {
    .metadata = {
      .uuid = uuid,
      .type = type,
      .slot = slot,
      .modifier_node_id = 0,
      .length = sizeof(credential_data),
      .modifier_type = MODIFIER_TYPE_LOCALLY
    },
    .data = credential_data
  };

  if (CC_UserCredential_add_credential(&credential) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    operation_result = true;
    app_log_info("Credential added with the following parameters:\r\n");
    app_log_append_info("\ttype: %d\r\n", credential.metadata.type);
    app_log_append_info("\tuuid: %d\r\n", credential.metadata.uuid);
    app_log_append_info("\tslot: %d\r\n", credential.metadata.slot);
    app_log_append_info("\tlength: %d\r\n", credential.metadata.length);
    app_log_append_info("\tmodifier_node_id: %d\r\n", credential.metadata.modifier_node_id);
    app_log_append_info("\tmodifier_type: %d\r\n", credential.metadata.modifier_type);
    app_log_append_info("\tdata: %s\r\n", credential.data);
  }
  return operation_result;
}

/**
 * @brief Modifies the name of a user.
 *
 * This function allows for modifying the name of a user in the database.
 * It takes an instance of u3c_user structure and the new name as parameters and updates the
 * user's name accordingly.
 *
 * @param user Instance of u3c_user structure.
 * @param new_name The new name to assign to the user.
 * @return Returns true if the user's name was successfully modified, or
 *         false value if an error occurred.
 */
static bool u3c_modify_user_name(u3c_user *user, char* name)
{
  bool operation_result = false;

  user->name_length = strlen(name);
  if ( CC_UserCredential_modify_user(user, (unsigned char*)name) == U3C_DB_OPERATION_RESULT_SUCCESS) {
    operation_result = true;
  }

  return operation_result;
}

/**
 * @brief Modifies the user type of a given user.
 *
 * This function is used to modify the user type of a specified user in the database.
 *
 * @param user Pointer to the user structure to be modified.
 * @param type The new user type to be assigned.
 * @return Returns true if the user type was successfully modified, false otherwise.
 */
static bool u3c_modify_user_type(u3c_user *user, char* type)
{
  bool operation_result = false;
  u3c_user_type new_user_type;
  new_user_type = u3c_convert_str_to_user_type(type);

  if (new_user_type != USER_TYPE_END) {
    user->type = new_user_type;
    if ( CC_UserCredential_modify_user(user, NULL) == U3C_DB_OPERATION_RESULT_SUCCESS) {
      operation_result = true;
    }
  }

  return operation_result;
}

/**
 * @brief Modifies the active status of a user.
 *
 * This function allows for modifying the active status of a user in the database.
 * The user's active status can be set to either active or inactive.
 *
 * @param user The u3c_user instance to modify.
 * @param active_status The new active status for the user.
 *
 * @return Returns true if the user's active status was successfully modified, or false if an error occurred.
 */
static bool u3c_modify_user_active(u3c_user *user, char* state)
{
  bool operation_result = true;
  bool active_state     = false;

  if (0 == strcmp(state, "true")) {
    active_state = true;
  } else if (0 == strcmp(state, "false")) {
    active_state = false;
  } else {
    app_log_error("State %s is not supported!\r\n", state);
    app_log_info("Valid values are:\r\n");
    app_log_append_info("\t- true\r\n");
    app_log_append_info("\t- false\r\n");
    operation_result = false;
  }

  if (operation_result) {
    user->active = active_state;
    if ( CC_UserCredential_modify_user(user, NULL) == U3C_DB_OPERATION_RESULT_SUCCESS) {
      operation_result = true;
    }
  }

  return operation_result;
}

/**
 * @brief Converts a string representation of a credential type to its corresponding u3c_credential_type value.
 *
 * This function takes a string as input and converts it to the corresponding u3c_credential_type value.
 * The string should represent a valid credential type.
 *
 * @param str The string representation of the credential type.
 * @return The u3c_credential_type value corresponding to the input string. CREDENTIAL_TYPE_NUMBER_OF_TYPES in case of unknown raw string.
 */
static u3c_credential_type u3c_convert_str_to_credential_type(unsigned char* str)
{
  u3c_credential_type credential_type = CREDENTIAL_TYPE_NUMBER_OF_TYPES;

  if (str) {
    if (0 == strcmp((char*)str, "pin")) {
      credential_type = CREDENTIAL_TYPE_PIN_CODE;
    } else if (0 == strcmp((char*)str, "password")) {
      credential_type = CREDENTIAL_TYPE_PASSWORD;
    } else {
      app_log_error("Credential type %s is not supported!\r\n", str);
      app_log_info("Supported typed are:\r\n");
      app_log_append_info("\t- pin");
      app_log_append_info("\t- password");
    }
  }

  return credential_type;
}

/**
 * @brief Converts a string to u3c_user_type type.
 *
 * This function takes a string as input and converts it to u3c_user_type type.
 * The conversion process involves parsing the string and extracting the relevant
 * information to create an instance of the user-defined type.
 *
 * @param str The string to be converted.
 * @return The converted u3c_user_type type. USER_TYPE_END in case of unknown raw string.
 */
static u3c_user_type u3c_convert_str_to_user_type(char* str)
{
  u3c_user_type user_type = USER_TYPE_END;

  if (0 == strcmp(str, "general")) {
    user_type = USER_TYPE_GENERAL;
  } else if (0 == strcmp(str, "programming")) {
    user_type = USER_TYPE_PROGRAMMING;
  } else if (0 == strcmp(str, "non_access")) {
    user_type = USER_TYPE_NON_ACCESS;
  } else if (0 == strcmp(str, "duress")) {
    user_type = USER_TYPE_DURESS;
  } else if (0 == strcmp(str, "disposable")) {
    user_type = USER_TYPE_DISPOSABLE;
  } else if (0 == strcmp(str, "expiring")) {
    user_type = USER_TYPE_EXPIRING;
  } else if (0 == strcmp(str, "remote_only")) {
    user_type = USER_TYPE_REMOTE_ONLY;
  } else {
    app_log_error("New type %s is not supported!\r\n", str);
    app_log_info("Valid values are\r\n");
    app_log_append_info("\t- general\r\n");
    app_log_append_info("\t- programming\r\n");
    app_log_append_info("\t- non_access\r\n");
    app_log_append_info("\t- duress\r\n");
    app_log_append_info("\t- disposable\r\n");
    app_log_append_info("\t- expiring\r\n");
    app_log_append_info("\t- remote_only\r\n");
  }

  return user_type;
}

/**
 * @brief Converts a string to a Credential attribute.
 *
 * This function takes a string as input and converts it to a Credential attribute.
 * The Credential attribute is a data structure that represents a specific attribute of a credential.
 *
 * @param str The string to be converted.
 * @return The converted Credential attribute.
 */
static u3c_credential_attribute u3c_convert_str_to_credential_attribute(char* str)
{
  u3c_credential_attribute attribute = CREDENTIAL_ATTRIBUTE_END;

  if (0 == strcmp(str, "data")) {
    attribute = CREDENTIAL_ATTRIBUTE_CREDENTIAL;
  } else if (0 == strcmp(str, "uuid")) {
    attribute = CREDENTIAL_ATTRIBUTE_UUID;
  } else if (0 == strcmp(str, "slot")) {
    attribute = CREDENTIAL_ATTRIBUTE_SLOT;
  } else {
    app_log_error("Credential attribute %s is not supported!\r\n", str);
    app_log_info("Valid values are\r\n");
    app_log_append_info("\t- data\r\n");
    app_log_append_info("\t- uuid\r\n");
    app_log_append_info("\t- slot\r\n");
  }

  return attribute;
}

/**
 * @brief Generates a UUID.
 *
 * Gets an availavle uuid from the database, which can be assigned to a new user.
 *
 * @return Returns an available uuid
 */
static uint16_t u3c_generate_uuid(void)
{
  static uint16_t uuid = 1;
  uint16_t max_uuid = 0;
  u3c_db_operation_result db_operation_result = U3C_DB_OPERATION_RESULT_ERROR;

  max_uuid = cc_user_credential_get_max_user_unique_idenfitiers();

  for (; uuid <= max_uuid; uuid++) {
    db_operation_result = CC_UserCredential_get_user(uuid, NULL, NULL);

    if (db_operation_result == U3C_DB_OPERATION_RESULT_FAIL_DNE) {
      break;
    } else if (db_operation_result == U3C_DB_OPERATION_RESULT_SUCCESS) {
      continue;
    } else {
      app_log_error("Database operation failed with %d error code!\r\n", db_operation_result);
      break;
    }
  }

  return uuid;
}

#endif // SL_CATALOG_ZW_CLI_COMMON_PRESENT
