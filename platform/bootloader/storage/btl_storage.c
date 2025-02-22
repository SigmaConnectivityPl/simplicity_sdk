/***************************************************************************//**
 * @file
 * @brief Storage component for the Gecko Bootloader.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "config/btl_config.h"

#include "api/btl_interface.h"
#include "btl_storage.h"
#ifndef BTL_CONFIG_FILE
#include "btl_storage_slot_cfg.h"
#include "btl_storage_cfg.h"
#endif

#include "core/btl_reset.h"
#include "core/btl_parse.h"
#include "core/btl_bootload.h"
#include "debug/btl_debug.h"

#if defined(CRYPTOACC_PRESENT)
#include "core/btl_util.h"
MISRAC_DISABLE
#include "sli_se_manager_mailbox.h"
#include "sli_se_manager_internal.h"
MISRAC_ENABLE
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif

// --------------------------------
// Prototypes

static int32_t installImageFromSlot(int32_t slotId);

// --------------------------------
// Function implementations

uint32_t storage_getBaseAddress(void)
{
#ifdef BTL_STORAGE_BASE_ADDRESS
  return BTL_STORAGE_BASE_ADDRESS;
#else
  return 0;
#endif
}

int32_t storage_main(void)
{
#if BTL_STORAGE_BOOTLOAD_LIST_LENGTH == 1
  return installImageFromSlot(0);
#else
  int32_t ret;

  int32_t slotIds[BTL_STORAGE_BOOTLOAD_LIST_LENGTH] = { -1 };

  ret = storage_getBootloadList(slotIds, BTL_STORAGE_BOOTLOAD_LIST_LENGTH);
  if ((ret != BOOTLOADER_OK)
      && (ret != BOOTLOADER_ERROR_BOOTLOAD_LIST_NO_LIST)) {
    BTL_DEBUG_PRINTLN("BI err");
    return ret;
  } else if (ret == BOOTLOADER_ERROR_BOOTLOAD_LIST_NO_LIST) {
    // Try to bootload in an sequential order
    for (uint8_t i = 0; i < BTL_STORAGE_BOOTLOAD_LIST_LENGTH; i++) {
      slotIds[i] = i;
    }
  }

  // Attempt to bootload given images in sequence
  for (size_t id = 0; id < BTL_STORAGE_BOOTLOAD_LIST_LENGTH; id++) {
    if (slotIds[id] == -1) {
      // Invalid slot ID; try the next one
      continue;
    }

    ret = installImageFromSlot(slotIds[id]);
    if (ret == BOOTLOADER_OK) {
      break;
    }
  }

  return ret;
#endif
}

static int32_t installImageFromSlot(int32_t slotId)
{
  BootloaderParserContext_t parseContext;
  int32_t ret;
  uint32_t deltaGBLLength = 0;

  BTL_DEBUG_PRINT("Slot: ");
  BTL_DEBUG_PRINT_WORD_HEX(slotId);
  BTL_DEBUG_PRINT_LF();

  // Get info about the image marked for bootloading
  storage_initParseSlot(slotId,
                        &parseContext,
                        sizeof(BootloaderParserContext_t));

  do {
    ret = storage_verifySlot(&parseContext, NULL);
  } while (ret == BOOTLOADER_ERROR_PARSE_CONTINUE);

  if (ret != BOOTLOADER_ERROR_PARSE_SUCCESS) {
    // Image in slot is non-existant or corrupt.
    // Continue to next image
    BTL_DEBUG_PRINTLN("Verify fail");
    return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
  }
#if defined (BTL_PARSER_SUPPORT_DELTA_DFU)
  deltaGBLLength = parseContext.parserContext.gblLength;
  if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_DELTA)
      && ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_SE)
          || (parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_BOOTLOADER))) {
    //Comes here only if the GBL has Bootloader/SE and a delta patch.
    //Extract and don't copy the firmware. Then continue with the rest of the images.
    BTL_DEBUG_PRINTLN("Image has delta.");
    uint32_t startOfAppSpace = BTL_APPLICATION_BASE;
    uint32_t pc = *(uint32_t *)(startOfAppSpace + 4);
    if (pc != 0xFFFFFFFF) {
      BTL_DEBUG_PRINTLN("Valid App in App Space. Start Reconstruction.");
      storage_bootloadApplicationFromSlot(
        parseContext.slotId,
        parseContext.imageProperties.application.version, deltaGBLLength);
    }
  }
#endif

#if (_SILICON_LABS_32B_SERIES == 1)
  // The upgrade address can be placed to not overlap with application images on Series-2.
  if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_BOOTLOADER)
      && !(parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_APPLICATION)) {
    BTL_DEBUG_PRINTLN("BL upg with no app");
    return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
  }
#endif

#if defined(SEMAILBOX_PRESENT) || defined(CRYPTOACC_PRESENT)
  if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_SE)
      && bootload_checkSeUpgradeVersion(parseContext.imageProperties.seUpgradeVersion)) {
#if defined(CRYPTOACC_PRESENT)
    // Check whether the SE handled the SLI_SE_COMMAND_APPLY_SE_IMAGE command during boot
    // and see if the SE image was corrupted and verification failed.
    // The only reponse message of interest is SLI_SE_RESPONSE_INVALID_SIGNATURE.
    // All the other cases, bootloader will try to apply the upgrade image.
    if (sli_vse_mailbox_read_executed_command() == SLI_SE_COMMAND_APPLY_SE_IMAGE) {
      sli_se_mailbox_command_t seCommand = SLI_SE_MAILBOX_COMMAND_DEFAULT(SLI_SE_COMMAND_APPLY_SE_IMAGE);
      sli_se_mailbox_response_t response = sli_vse_mailbox_ack_command(&seCommand);
      BTL_DEBUG_PRINT("SE response: ");
      BTL_DEBUG_PRINT_WORD_HEX(response);
      BTL_DEBUG_PRINT_LF();
      if (response == SLI_SE_RESPONSE_INVALID_SIGNATURE) {
        return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
      }
    }
#endif
    if (storage_upgradeSeFromSlot(parseContext.slotId)) {
      // SE upgrade should be applied
#if defined(BOOTLOADER_SE_UPGRADE_NO_STAGING) \
      && (BOOTLOADER_SE_UPGRADE_NO_STAGING == 1)
      const BootloaderStorageSlot_t storageSlots[] = BTL_STORAGE_SLOTS;
      const uint32_t upgradeAddress = storageSlots[slotId].address
                                      + parseContext.parserContext.offsetOfSeUpgradeTag;
#else
      const uint32_t upgradeAddress = BTL_UPGRADE_LOCATION;
#endif
      if (!bootload_commitSeUpgrade(upgradeAddress)) {
        BTL_DEBUG_PRINTLN("SE upgrade commit fail");
        return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
      }
    } else {
      BTL_DEBUG_PRINTLN("SE upgrade parse fail");
    }
  }
#endif // SEMAILBOX_PRESENT || CRYPTOACC_PRESENT

  if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_BOOTLOADER)) {
    BTL_DEBUG_PRINT("BL upg ");
    BTL_DEBUG_PRINT_WORD_HEX(mainBootloaderTable->header.version);
    BTL_DEBUG_PRINT(" -> ");
    BTL_DEBUG_PRINT_WORD_HEX(parseContext.imageProperties.bootloaderVersion);
    BTL_DEBUG_PRINT_LF();
  }

  if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_BOOTLOADER)
      && (parseContext.imageProperties.bootloaderVersion
          > mainBootloaderTable->header.version)) {
    // This is a bootloader upgrade, and we also have an application
    // available for after the bootloader upgrade is complete
    if (storage_bootloadBootloaderFromSlot(
          parseContext.slotId,
          parseContext.imageProperties.bootloaderVersion)) {
      if (!bootload_commitBootloaderUpgrade(BTL_UPGRADE_LOCATION, parseContext.imageProperties.bootloaderUpgradeSize)) {
        // Bootloader upgrade failed; not a valid image
        BTL_DEBUG_PRINTLN("Btl upgrade commit fail");
      }
    } else {
      // Bootloader upgrade failed; not a valid image
      BTL_DEBUG_PRINTLN("Btl upgrade fail");
    }
    return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
  } else {
#if (defined(BOOTLOADER_SE_UPGRADE_NO_STAGING)  \
    && (BOOTLOADER_SE_UPGRADE_NO_STAGING == 0)) \
    || !defined(BOOTLOADER_SE_UPGRADE_NO_STAGING)
    // This should be an application upgrade
    if (!(parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_APPLICATION)) {
      // ...but there is no app in the GBL
      BTL_DEBUG_PRINTLN("No app in slot");
      // Continue to next image
      return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
    }
#endif // BOOTLOADER_SE_UPGRADE_NO_STAGING

    if ((parseContext.imageProperties.contents & BTL_IMAGE_CONTENT_APPLICATION)
        && !storage_bootloadApplicationFromSlot(
          parseContext.slotId,
          parseContext.imageProperties.application.version, deltaGBLLength)) {
      // App upgrade failed.
      BTL_DEBUG_PRINTLN("App upgrade fail");
      // Continue to next image
      return BOOTLOADER_ERROR_STORAGE_BOOTLOAD;
    }
    // Application was updated. Reboot into new image.
  }

  return BOOTLOADER_OK;
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
