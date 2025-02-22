/***************************************************************************//**
 * @file
 * @brief APIs and defines for the GBCS Device Log plugin.
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

#ifndef GBCS_DEVICE_LOG_H_
#define GBCS_DEVICE_LOG_H_

#include "gbcs-device-log-config.h"

/**
 * @defgroup gbcs-device-log GBCS Device Log
 * @ingroup component
 * @brief API and Callbacks for the gbcs-device-log Component
 *
 * Silicon Labs implementation of the GBCS Device Log. Note that, although
 * this supports populating the device log with a set of devices,
 * this is only intended for testing purposes and should not be thought of as a
 * complete provisioning solution. The real provisioning solution should use the
 * API provided by this component to add and remove items from the device log.
 *
 */

/**
 * @addtogroup gbcs-device-log
 * @{
 */

// Printing macros for plugin: GBCS Device Log
#define sl_zigbee_af_gbcs_device_log_print(...)    sl_zigbee_af_app_print(__VA_ARGS__)
#define sl_zigbee_af_gbcs_device_log_println(...)  sl_zigbee_af_app_println(__VA_ARGS__)
#define sl_zigbee_af_gbcs_device_log_debug_exec(x)  sl_zigbee_af_app_debug_exec(x)

typedef enum {
  SL_ZIGBEE_AF_GBCS_GSME_DEVICE_TYPE   = 0x00,
  SL_ZIGBEE_AF_GBCS_ESME_DEVICE_TYPE   = 0x01,
  SL_ZIGBEE_AF_GBCS_CHF_DEVICE_TYPE    = 0x02,
  SL_ZIGBEE_AF_GBCS_GPF_DEVICE_TYPE    = 0x03,
  SL_ZIGBEE_AF_GBCS_HCALCS_DEVICE_TYPE = 0x04,
  SL_ZIGBEE_AF_GBCS_PPMID_DEVICE_TYPE  = 0x05,
  SL_ZIGBEE_AF_GBCS_TYPE2_DEVICE_TYPE  = 0x06,
} sl_zigbee_af_g_b_c_s_device_type_t;

typedef struct {
  sl_zigbee_af_g_b_c_s_device_type_t deviceType;
} sl_zigbee_af_g_b_c_s_device_log_info_t;

/**
 * @name API
 * @{
 */

/**
 * @brief Clear all entries from the GBCS Device Log.
 */
void sl_zigbee_af_gbcs_device_log_clear(void);

/**
 * @brief Reset the GBCS Device Log to include only those entries configured with the plugin.
 *
 * This function will clear the device log then add each of the devices configured
 * configured with the plugin.
 */
void sl_zigbee_af_gbcs_device_log_reset(void);

/**
 * @brief Get the number of entries in the GBCS Device Log.
 *
 * @return The number of entries currently in the GBCS Device Log.
 */
uint8_t sl_zigbee_af_gbcs_device_log_count(void);

/**
 * @brief Add or update an entry within the GBCS Device Log.
 *
 * This function is used to add or update an entry within the GBCS Device Log.
 *
 * @param deviceId The EUI64 of the device to be added or updated.
 * @param deviceInfo A pointer to the structure containing the deviceType of the
 * device to be added or updated.
 * @return True if successfully added or updated, false if there is insufficient
 * room in the device log to add the new entry. The size of the device log is
 * controlled by the "Device Log Size" plugin configuration parameter.
 */
bool sl_zigbee_af_gbcs_device_log_store(sl_802154_long_addr_t deviceId,
                                        sl_zigbee_af_g_b_c_s_device_log_info_t *deviceInfo);

/**
 * @brief Remove an entry from the GBCS Device Log.
 *
 * This function can be used to remove an entry within the GBCS Device Log.
 *
 * @param deviceId The EUI64 of the device to be removed.
 * @return True if successfully removed, false if the device is not currently
 * in the GBCS Device Log.
 */
bool sl_zigbee_af_gbcs_device_log_remove(sl_802154_long_addr_t deviceId);

/**
 * @brief Get the device information about an entry in the GBCS Device Log.
 *
 * This function can be used to retrieve information pertaining to a device
 * in the GBCS Device Log. Note that if a single device implements two
 * device types (i.e., comms hub implements both the CHF and GPF),
 * only the first device found is returned.
 *
 * @param deviceId The EUI64 of the device for which information is requested.
 * @return True if device is in the device log, false otherwise. The contents of
 * the deviceInfo structure will be populated when true is returned.
 */
bool sl_zigbee_af_gbcs_device_log_get(sl_802154_long_addr_t deviceId,
                                      sl_zigbee_af_g_b_c_s_device_log_info_t *deviceInfo);

/**
 * @brief Check whether the given device exists within the GBCS Device Log.
 *
 * This function can be used to check if the given device and type exists within
 * the GBCS Device Log.
 *
 * @param deviceId The EUI64 of the device in question
 * @param deviceType The device type of the device in question.
 * @return True if device is in the device log, false otherwise.
 */
bool sl_zigbee_af_gbcs_device_log_exists(sl_802154_long_addr_t deviceId,
                                         sl_zigbee_af_g_b_c_s_device_type_t deviceType);

/**
 * @brief Print all entries in the GBCS Device Log.
 */
void sl_zigbee_af_gbcs_device_log_print_entries(void);

/**
 * @brief Indicate whether the given device type is a sleepy device.
 *
 * @param deviceType The GBCS device type to be checked.
 * @return True if the deviceType is a sleepy device, false otherwise.
 */
#define sl_zigbee_af_gbcs_device_log_is_sleepy_type(deviceType) ((deviceType) == SL_ZIGBEE_AF_GBCS_GSME_DEVICE_TYPE)

/**
 * @brief Retrieve the device from the log at the specified index.
 *
 * @param index The absolute index into the device log.
 * @param returnDeviceId A pointer where the EUI64 return data will be written.
 * @param returnDeviceInfo A pointer where the device info return data will be written.
 * @return True if the index has a valid entry.
 *   false otherwise.
 */
bool sl_zigbee_af_gbcs_device_log_retrieve_by_index(uint8_t index,
                                                    sl_802154_long_addr_t returnDeviceId,
                                                    sl_zigbee_af_g_b_c_s_device_log_info_t* returnDeviceInfo);

/**
 * @ brief Return the maximum size of the device log, including currently empty entries.
 *
 * @return An uint8_t indicating the maximum size.
 */
uint8_t sl_zigbee_af_gbcs_device_log_max_size(void);
/** @} */ // end of name API

/**
 * @name Callbacks
 * @{
 */

/**
 * @defgroup gbcs_device_log_cb GBCS Device Log
 * @ingroup af_callback
 * @brief Callbacks for GBCS Device Log Component
 *
 */

/**
 * @addtogroup gbcs_device_log_cb
 * @{
 */

/** @brief Device removed.
 *
 * This callback is called by the plugin when a device is removed from the
 * device log.
 *
 * @param deviceId Identifier of the device removed Ver.: always
 */
void sl_zigbee_af_gbcs_device_log_device_removed_cb(sl_802154_long_addr_t deviceId);
/** @} */ // end of gbcs_device_log_cb
/** @} */ // end of name Callbacks
/** @} */ // end of gbcs-device-log

#endif /* GBCS_DEVICE_LOG_H_ */
