/***************************************************************************//**
 * @file
 * @brief This code will load a file from disk into the 'Simple Storage' plugin.
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
#include "app/framework/plugin/ota-common/ota.h"

#include "app/util/serial/sl_zigbee_command_interpreter.h"

#if defined(SL_ZIGBEE_TEST)

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

//------------------------------------------------------------------------------
// Globals

#define MAX_READ_SIZE 512
#define MAX_PATH_SIZE 512

//------------------------------------------------------------------------------
// Forward Declarations
//------------------------------------------------------------------------------

static bool loadFileIntoOtaStorage(char* file)
{
  FILE* fh = fopen(file, "rb");
  char cwd[MAX_PATH_SIZE];

  assert(NULL != getcwd(cwd, MAX_PATH_SIZE));

  sl_zigbee_af_ota_bootload_cluster_flush();
  otaPrintln("Current directory: '%s'", cwd);
  sl_zigbee_af_ota_bootload_cluster_flush();

  if (fh == NULL) {
    otaPrintln("Failed to open file: %p",
               strerror(errno));
    return false;
  }

  struct stat buffer;
  if (0 != stat(file, &buffer)) {
    otaPrintln("Failed to stat() file: %p",
               strerror(errno));
    return false;
  }

  sl_zigbee_af_ota_storage_status_t status = sl_zigbee_af_ota_storage_clear_temp_data_cb();
  if (status != SL_ZIGBEE_AF_OTA_STORAGE_SUCCESS) {
    otaPrintln("Failed to delete existing OTA file.");
    return false;
  }

  off_t offset = 0;
  while (offset < buffer.st_size) {
    uint8_t data[MAX_READ_SIZE];
    off_t readSize = (buffer.st_size - offset > MAX_READ_SIZE
                      ? MAX_READ_SIZE
                      : buffer.st_size - offset);
    size_t readAmount = fread(data, 1, readSize, fh);
    if (readAmount != (size_t) readSize) {
      otaPrintln("Failed to read %d bytes from file at offset 0x%4X",
                 readSize,
                 offset);
      status = SL_ZIGBEE_AF_OTA_STORAGE_ERROR;
      goto loadStorageDone;
    }

    status = sl_zigbee_af_ota_storage_write_temp_data_cb(offset,
                                                         readAmount,
                                                         data);
    if (status != SL_ZIGBEE_AF_OTA_STORAGE_SUCCESS) {
      otaPrintln("Failed to load data into temp storage.");
      goto loadStorageDone;
    }

    offset += readAmount;
  }

  status = sl_zigbee_af_ota_storage_finish_download_cb(offset);

  if (status == SL_ZIGBEE_AF_OTA_STORAGE_SUCCESS) {
    uint32_t totalSize;
    uint32_t offset;
    sl_zigbee_af_ota_image_id_t id;
    status = sl_zigbee_af_ota_storage_check_temp_data_cb(&offset,
                                                         &totalSize,
                                                         &id);

    if (status != SL_ZIGBEE_AF_OTA_STORAGE_SUCCESS) {
      otaPrintln("Failed to validate OTA file.");
      goto loadStorageDone;
    }

    otaPrintln("Loaded image successfully.");
  }

  loadStorageDone:
  fclose(fh);
  return status;
}
#else
static bool loadFileIntoOtaStorage(char* file)
{
  return false;
}
#endif // SL_ZIGBEE_TEST

#define MAX_FILENAME_SIZE   255

#include "sl_cli.h"

void sli_zigbee_af_ota_load_file_command(sl_cli_command_arg_t *args)
{
  char filename[MAX_FILENAME_SIZE];
  uint8_t length;
  uint8_t *buffer = sl_zigbee_cli_get_argument_string_and_length(args, 0, &length);
  if (length >= MAX_FILENAME_SIZE) {
    otaPrintln("OTA ERR: filename '%s' is too long (max %d chars)",
               buffer,
               MAX_FILENAME_SIZE - 1);
    return;
  }
  memmove(filename, buffer, length);
  filename[length] = '\0';
  otaPrintln("Loading from file: '%s'", filename);

  loadFileIntoOtaStorage(filename);
}
