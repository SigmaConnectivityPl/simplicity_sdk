/***************************************************************************//**
 * @file
 * @brief This is an integration of the simple OTA storage driver with the low-level
 * EEPROM driver.
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
#include "app/framework/util/util.h"
#include "app/framework/plugin/ota-common/ota.h"
#include "app/framework/plugin/ota-storage-common/ota-storage.h"

#include "eeprom.h"

//#define DEBUG_PRINT
#define OTA_STORAGE_EEPROM_INTERNAL_HEADER
#include "ota-storage-eeprom.h"
#undef OTA_STORAGE_EEPROM_INTERNAL_HEADER

#if defined(SL_ZIGBEE_TEST)
 #include "hal/micro/unix/simulation/fake-eeprom.h"
#endif // SL_ZIGBEE_TEST

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif

#ifdef SL_CATALOG_SLOT_MANAGER_PRESENT
#include "slot-manager.h"
#endif // SL_CATALOG_SLOT_MANAGER_PRESENT

//------------------------------------------------------------------------------
// Prototypes

#if defined(DEBUG_PRINT)
static void printImageInfoStartData(void);
static void printDataBlock(const uint8_t* block);
#else
  #define printImageInfoStartData()
  #define printDataBlock(x)
#endif // DEBUG_PRINT

void calculateSlotAndEepromOffsets();
void sli_eeprom_info_command(void);

//------------------------------------------------------------------------------
// Globals

#ifdef READ_MODIFY_WRITE_SUPPORT
  #define COMPILED_FOR_READ_MODIFY_WRITE true
#else
  #define COMPILED_FOR_READ_MODIFY_WRITE false
#endif

// For debugging only
#define DATA_SIZE 48

// Slot interfacing macros
#if (SLOT_STRATEGY == USE_FIRST_SLOT)
 #define BOOTLOADER_STORAGE_SUPPORT_TEXT  "Use first slot"
#elif (SLOT_STRATEGY == USE_LAST_SLOT)
 #define BOOTLOADER_STORAGE_SUPPORT_TEXT  "Use last slot"
#elif (SLOT_STRATEGY == USE_SPECIFIC_SLOT)
 #define BOOTLOADER_STORAGE_SUPPORT_TEXT  "Use specific slot"
#else
 #define BOOTLOADER_STORAGE_SUPPORT_TEXT  "Do not use slots"
#endif // SLOT_STRATEGY == (USE_FIRST_SLOT | USE_LAST_SLOT | USE_SPECIFIC_SLOT)

// Only needed for Page-erase-required flash parts.
sl_zigbee_af_event_t sl_zigbee_af_ota_storage_simple_eeprom_page_erase_event;

static uint32_t gOtaEepromSize    = EEPROM_END - EEPROM_START;
static uint32_t gOtaStorageStart  = EEPROM_START;
static uint32_t gOtaStorageEnd    = EEPROM_END;
static uint32_t gOtaSlotToUse     = INVALID_SLOT;

#if defined(SOC_BOOTLOADING_SUPPORT)
static uint32_t gOtaImageInfoStart = EEPROM_END \
                                     - MAX_IMAGE_INFO_AND_OTA_HEADER_SIZE;
#else
static uint32_t gOtaImageInfoStart = EEPROM_START;
#endif // SOC_BOOTLOADING_SUPPORT

//------------------------------------------------------------------------------

#if defined(SL_ZIGBEE_TEST)

#ifndef SL_ZIGBEE_TEST_EEPROM_WORD_SIZE
#define SL_ZIGBEE_TEST_EEPROM_WORD_SIZE MAX_WORD_SIZE
#endif

static void setup_fake_eeprom_for_simulation(void)
{
  setupFakeEeprom(gOtaEepromSize,
                  gOtaStorageStart,    // offset
                  2048,                // page size
                  sl_util_af_eeprom_info()->pageEraseMs,
                  (COMPILED_FOR_READ_MODIFY_WRITE == false),
                  SL_ZIGBEE_TEST_EEPROM_WORD_SIZE);      // word size
  sli_eeprom_fake_eeprom_callback();
}

#endif // SL_ZIGBEE_TEST

uint32_t sli_zigbee_af_ota_storage_read_int32u_from_eeprom(uint32_t realOffset)
{
  uint8_t value[4];
  sl_zigbee_af_eeprom_read(realOffset, value, 4);
  return (value[0]
          + ((uint32_t)value[1] << 8)
          + ((uint32_t)value[2] << 16)
          + ((uint32_t)value[3] << 24));
}

void sli_zigbee_af_ota_storage_write_int32u_to_eeprom(uint32_t value, uint32_t realOffset)
{
  uint32_t oldValue = sli_zigbee_af_ota_storage_read_int32u_from_eeprom(realOffset);
  if (oldValue != value) {
    uint8_t data[4];
    data[0] = value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);

    sl_zigbee_af_eeprom_write(realOffset, data, 4);
  }
}

#if defined (SOC_BOOTLOADING_SUPPORT)

uint32_t sli_zigbee_af_get_ebl_start_offset(void)
{
  return sli_zigbee_af_ota_storage_read_int32u_from_eeprom(gOtaImageInfoStart
                                                           + EBL_START_OFFSET_INDEX);
}

static void setEblStartOffset(uint32_t eblStart)
{
  debugPrint("Writing EBL start offset of 0x%4X to EEPROM offset 0x%4X",
             eblStart,
             gOtaImageInfoStart + EBL_START_OFFSET_INDEX);
  debugFlush();
  sli_zigbee_af_ota_storage_write_int32u_to_eeprom(eblStart,
                                                   gOtaImageInfoStart
                                                   + EBL_START_OFFSET_INDEX);

  #if defined(DEBUG_PRINT)
  {
    uint32_t offset = sli_zigbee_af_get_ebl_start_offset();
    debugPrint("EBL Start Offset: 0x%4X", offset);
  }
  #endif // DEBUG_PRINT
}
#endif // SOC_BOOTLOADING_SUPPORT

bool sl_zigbee_af_ota_storage_driver_init_cb(void)
{
  #if defined(SL_ZIGBEE_TEST)
  setup_fake_eeprom_for_simulation();
  #endif

  // First, if we're using slots, calculate EEPROM start, end, and length
  calculateSlotAndEepromOffsets();

  // Older drivers do not have an EEPROM info structure that we can reference
  // so we must just assume they are okay.
  if (sl_util_af_eeprom_info() != NULL) {
    assert(sl_util_af_eeprom_info()->partSize >= gOtaEepromSize);
  }
  sli_zigbee_af_ota_storage_eeprom_init();

  return true;
}

void sli_zigbee_af_ota_storage_simple_eeprom_init_callback(uint8_t init_level)
{
  (void)init_level;

  sl_zigbee_af_event_init(eraseEvent,
                          sl_zigbee_af_ota_storage_simple_eeprom_page_erase_event_handler);
}

// Returns true if the operation crosses the break in the OTA image
// due to Layout 2.  Otherwise returns false.  Modifies
// the OTA offset and turns it into the real EEPROM offset.
// This will be based on the start offset of the EEPROM (since the user
// may have allocated a subset of the EEPROM for OTA and not positioned
// the OTA data at offset 0), and after the image info meta-data.
bool sli_zigbee_af_ota_storage_driver_get_real_offset(uint32_t* offset,
                                                      uint32_t* length)
{
  bool spansBreak = false;
  uint32_t realOffset = gOtaImageInfoStart + OTA_HEADER_INDEX + *offset;

#if defined(SOC_BOOTLOADING_SUPPORT)
  uint32_t eblOffset = sli_zigbee_af_get_ebl_start_offset();

  if (*offset < eblOffset) {
    // Layout 2, before the break in the OTA image, but spans the break.
    if ((*offset + *length) > eblOffset) {
      spansBreak = true;
      *length = eblOffset - *offset;
    } // Else
      //   Data before the break in the OTA image, but doesn't span the break.
  } else {
    // Layout 2 starting after the break in the image
    realOffset = gOtaStorageStart + *offset - eblOffset;
  }
#else
  // Layout 1.  Do nothing more than what we have done already.
#endif // SOC_BOOTLOADING_SUPPORT

  *offset = realOffset;
  return spansBreak;
}

// This ugly code must handle the worst case scenario
// where we are trying to read/write a block of data
// in Layout 2 which spans the break in the OTA image.
// In that case we must perform 2 read/write operations
// to make it work.

static bool readWritePrimitive(bool read,
                               const uint8_t* writeData,
                               uint32_t offset,
                               uint32_t length,
                               uint8_t* readData)
{
  // Because the EEPROM code only handles the length as a 16-bit number,
  // we catch that error case.
  if (length > 65535) {
    return false;
  }

  uint8_t count = 1;
  uint8_t i;
  uint32_t realLength = length;
  uint32_t realOffset = offset;

  debugPrint("readWritePrimitive(): OTA offset 0x%4X, length %l", offset, length);

  bool spansBreak = sli_zigbee_af_ota_storage_driver_get_real_offset(&realOffset, &realLength);
  if (spansBreak) {
    count = 2;
  }

  for (i = 0; i < count; i++) {
    uint8_t status;

    debugFlush();
    debugPrint("%p realOffset: 0x%4X, realLength: %l",
               (read ? "read" : "write"),
               realOffset,
               realLength);
    debugFlush();

    if (read) {
      status = sl_zigbee_af_eeprom_read(realOffset, readData, (uint16_t)realLength);
    } else {
      status = sl_zigbee_af_eeprom_write(realOffset, writeData, (uint16_t)realLength);

      // If we're writing the end of the OTA header and the size is not word-
      // aligned, then we need to pad the number of bytes necessary to write the
      // whole word, otherwise the final bytes will stay in a partial word cache
      // forever and never be written to flash
      if ((status == SL_STATUS_OK)
          && spansBreak
          && (i == 0)
          && ((realLength + realOffset) % sl_util_af_eeprom_get_word_size())) {
        uint8_t remainingLen = realLength - (realLength - sl_util_af_eeprom_get_word_size());
        uint8_t zeroData[MAX_WORD_SIZE] = { 0 };
        status = sl_zigbee_af_eeprom_write(realOffset + realLength,
                                           zeroData,
                                           remainingLen);
      }
    }

    if (status != EEPROM_SUCCESS) {
      return false;
    }

    if (count > 1) {
      // Layout 2 only, and spans the break
      realOffset = gOtaStorageStart;
      if (read) {
        readData += realLength;
      } else {
        writeData += realLength;
      }
      realLength = length - realLength;
    }
  }
  return true;
}

// NOTE:  The magic number here is the "Ember" magic number.
//   It is not the same as the OTA file magic number.
//   It is used solely to verify the validity of the
//   meta-data stored ahead of the OTA file.
bool sli_zigbee_af_ota_storage_check_download_meta_data(void)
{
  uint8_t magicNumberExpected[] = { MAGIC_NUMBER, VERSION_NUMBER };
  uint8_t magicNumberActual[MAGIC_NUMBER_SIZE + VERSION_NUMBER_SIZE];

  sl_zigbee_af_eeprom_read(gOtaImageInfoStart + MAGIC_NUMBER_OFFSET,
                           magicNumberActual,
                           MAGIC_NUMBER_SIZE + VERSION_NUMBER_SIZE);
  if (0 != memcmp(magicNumberExpected,
                  magicNumberActual,
                  MAGIC_NUMBER_SIZE + VERSION_NUMBER_SIZE)) {
    debugPrint("Magic Number or version for download meta-data is invalid");
    debugFlush();
    return false;
  }

  return true;
}

// NOTE:  The magic number referenced here is the "Ember" Magic number.
// See comment above "sli_zigbee_af_ota_storage_check_download_meta_data()".
void sli_zigbee_af_ota_storage_write_download_meta_data(void)
{
  uint8_t magicNumber[] = { MAGIC_NUMBER, VERSION_NUMBER };
  debugPrint("Writing download meta-data (magic number and version)");
  debugFlush();
  sl_zigbee_af_eeprom_write(gOtaImageInfoStart + MAGIC_NUMBER_OFFSET,
                            magicNumber,
                            MAGIC_NUMBER_SIZE + VERSION_NUMBER_SIZE);
}

bool sl_zigbee_af_ota_storage_driver_read_cb(uint32_t offset,
                                             uint32_t length,
                                             uint8_t* returnData)
{
  return readWritePrimitive(true,   // read?
                            NULL,   // writeData pointer
                            offset,
                            length,
                            returnData);
}

static bool socBootloaderSupportWriteHandler(const uint8_t* dataToWrite,
                                             uint32_t offset,
                                             uint32_t length)
{
#if defined(SOC_BOOTLOADING_SUPPORT)
  uint16_t headerLength;
  debugPrint("socBootloaderSupportWriteHandler()");
  if (offset == 0) {
    if (length < (HEADER_LENGTH_OFFSET + HEADER_LENGTH_FIELD_LENGTH)) {
      // The expectation is that the first write of download data has at least
      // the header length in it.  Otherwise we can't determine where the EBL
      // starting point is.
      debugPrint("Write to offset 0 is too short!  Must be at least %d bytes",
                 HEADER_LENGTH_OFFSET + HEADER_LENGTH_FIELD_LENGTH);
      return false;
    }

    headerLength = ((dataToWrite[HEADER_LENGTH_OFFSET]
                     + (dataToWrite[HEADER_LENGTH_OFFSET + 1] << 8))
                    + TAG_OVERHEAD);

    setEblStartOffset(headerLength);
  }
#endif // SOC_BOOTLOADING_SUPPORT

  return true;
}

bool sl_zigbee_af_ota_storage_driver_write_cb(const uint8_t* dataToWrite,
                                              uint32_t offset,
                                              uint32_t length)
{
  if (!socBootloaderSupportWriteHandler(dataToWrite,
                                        offset,
                                        length)) {
    return false;
  }

  if (readWritePrimitive(false,        // read?
                         dataToWrite,
                         offset,
                         length,
                         NULL)) {      // readData pointer
    sli_zigbee_af_storage_eeprom_update_download_offset(offset + length,
                                                        false); // final offset?
    return true;
  }
  return false;
}

void sl_zigbee_af_ota_storage_driver_download_finish_cb(uint32_t finalOffset)
{
  debugPrint("Noting final download offset 0x%4X", finalOffset);
  sli_zigbee_af_storage_eeprom_update_download_offset(finalOffset,
                                                      true); // final offset?
  sl_zigbee_af_eeprom_flush_saved_partial_writes();
  return;
}

uint32_t sl_zigbee_af_ota_storage_driver_max_download_size_cb(void)
{
  return (gOtaEepromSize - MAX_IMAGE_INFO_AND_OTA_HEADER_SIZE);
}

void calculateSlotAndEepromOffsets()
{
#ifdef SL_CATALOG_SLOT_MANAGER_PRESENT
  uint8_t  status;
  uint32_t numSlots;
  uint32_t defaultSlotStartAddress, defaultSlotEndAddress;
  SlotManagerSlotInfo_t slotInfo;

  // Find out which slot to use (if applicable) and the storage start and end
  if (SLOT_STRATEGY != DO_NOT_USE_SLOTS) {
    status = sl_util_af_slot_manager_get_number_of_slots(&numSlots);

    if ((SL_STATUS_OK == status) && (numSlots != 0)) {
      if (SLOT_STRATEGY == USE_FIRST_SLOT) {
        gOtaSlotToUse = 0;
      } else if (SLOT_STRATEGY == USE_LAST_SLOT) {
        gOtaSlotToUse = numSlots - 1;
      } else { // (SLOT_STRATEGY == USE_SPECIFIC_SLOT)
        gOtaSlotToUse = SL_ZIGBEE_AF_PLUGIN_OTA_STORAGE_SIMPLE_EEPROM_SLOT_TO_USE;
      }

      status = sl_util_af_slot_manager_get_slot_info(gOtaSlotToUse, &slotInfo);

      if (SL_STATUS_OK == status) {
        // Ensure that the slot is ok to use
        assert(gOtaSlotToUse < numSlots);

        gOtaStorageStart   = slotInfo.slotStorageInfo.address;
        gOtaStorageEnd     = gOtaStorageStart + slotInfo.slotStorageInfo.length;
        gOtaEepromSize     = gOtaStorageEnd - gOtaStorageStart;
#if defined(SOC_BOOTLOADING_SUPPORT)
        gOtaImageInfoStart = gOtaStorageEnd
                             - MAX_IMAGE_INFO_AND_OTA_HEADER_SIZE;
#else
        gOtaImageInfoStart = gOtaStorageStart;
#endif // SOC_BOOTLOADING_SUPPORT
      }
    }

    if (SL_STATUS_OK != status) {
      otaPrintln("OTA Simple Storage EEPROM warning: could not get slot info "
                 "for slot %d (error 0x%x). Defaulting to specified address "
                 "offsets", gOtaSlotToUse, status);
    } else if (0 == numSlots) {
      // A Gecko bootloader is on the chip but no slots are configured. OTA
      // bootloading can't work at all and this is a misconfiguration, so assert
      assert(0);
    }
  } else { // DO_NOT_USE_SLOTS
    // Ensure that the addresses selected will work with OTA bootloading
    status = sl_util_af_slot_manager_get_number_of_slots(&numSlots);

    if ((SL_STATUS_OK == status) && (numSlots != 0)) {
      // Without slot support, the OTA bootloading code defaults to using slot 0

      status = sl_util_af_slot_manager_get_slot_info(DEFAULT_SLOT, &slotInfo);

      if (SL_STATUS_OK == status) {
        // Ensure that the chosen addresses match the default slot's addresses
        defaultSlotStartAddress = slotInfo.slotStorageInfo.address;
        defaultSlotEndAddress   = defaultSlotStartAddress
                                  + slotInfo.slotStorageInfo.length;

        // An assert here means you chose Do Not Use Slots in the OTA EEPROM
        // plugin and you have a Gecko bootloader on the chip whose slot
        // addresses do not match up with what you chose in your OTA EEPROM
        // plugin. OTA cannot work so assert.
        assert(gOtaStorageStart == defaultSlotStartAddress);
        assert(gOtaStorageEnd   <= defaultSlotEndAddress);
      }
    }
  }
#endif // SL_CATALOG_SLOT_MANAGER_PRESENT
}

uint32_t sli_zigbee_af_ota_storage_get_slot(void)
{
  return gOtaSlotToUse;
}

uint32_t otaStorageEepromGetStorageStartAddress()
{
  return gOtaStorageStart;
}

uint32_t otaStorageEepromGetStorageEndAddress()
{
  return gOtaStorageEnd;
}

uint32_t otaStorageEepromGetImageInfoStartAddress()
{
  return gOtaImageInfoStart;
}

void sli_zigbee_af_ota_storage_driver_info_print(void)
{
  uint32_t downloadOffset =
    sl_zigbee_af_ota_storage_driver_retrieve_last_stored_offset_cb();

  otaPrintln("Storage Driver:             OTA Simple Storage EEPROM Plugin");
  otaPrintFlush();
  otaPrintln("Read Modify Write Support:  " READ_MODIFY_WRITE_SUPPORT_TEXT);
  otaPrintFlush();
  otaPrintln("SOC Bootloading Support:    " SOC_BOOTLOADING_SUPPORT_TEXT);
  otaPrintFlush();
  otaPrintln("Bootloader Storage Support: " BOOTLOADER_STORAGE_SUPPORT_TEXT);
  otaPrintFlush();
  if (SLOT_STRATEGY != DO_NOT_USE_SLOTS) {
    otaPrintln("Slot to use:                %d", gOtaSlotToUse);
    otaPrintFlush();
  }
  otaPrintln("Current Download Offset:    0x%4X", downloadOffset);

#if defined(SOC_BOOTLOADING_SUPPORT)
  otaPrintFlush();
  otaPrintln("EBL Start Offset:           0x%4X", sli_zigbee_af_get_ebl_start_offset());
  otaPrintFlush();
#endif // SOC_BOOTLOADING_SUPPORT

  otaPrintln("EEPROM Start:               0x%4X", gOtaStorageStart);
  otaPrintFlush();
  otaPrintln("EEPROM End:                 0x%4X", gOtaStorageEnd);
  otaPrintFlush();
  otaPrintln("Image Info Start:           0x%4X", gOtaImageInfoStart);
  otaPrintFlush();
  otaPrintln("Save Rate (bytes)           0x%4X", SAVE_RATE);
  otaPrintFlush();
  otaPrintln("Offset of download offset   0x%4X", gOtaImageInfoStart
             + SAVED_DOWNLOAD_OFFSET_INDEX);
  otaPrintFlush();
  otaPrintln("Offset of EBL offset:       0x%4X", gOtaImageInfoStart
             + EBL_START_OFFSET_INDEX);
  otaPrintFlush();
  otaPrintln("Offset of image start:      0x%4X", gOtaImageInfoStart
             + OTA_HEADER_INDEX);
  otaPrintFlush();

#if defined(DEBUG_PRINT)
  {
    uint8_t data[DATA_SIZE];

    otaPrintln("\nData at EEPROM Start");
    sl_zigbee_af_eeprom_read(gOtaStorageStart, data, DATA_SIZE);
    sl_zigbee_af_print_cert(data);  // certs are 48 bytes long
    otaPrintFlush();
  }
  printImageInfoStartData();
#endif // DEBUG_PRINT

  sli_eeprom_info_command();
}

#if defined(DEBUG_PRINT)

static void printImageInfoStartData(void)
{
  uint8_t data[DATA_SIZE];
  uint8_t i;
  uint32_t maxSize = (MAX_IMAGE_INFO_AND_OTA_HEADER_SIZE < 128
                      ? MAX_IMAGE_INFO_AND_OTA_HEADER_SIZE
                      : 128);
  otaPrintln("\nData at Image Info Start");
  otaPrintFlush();

  for (i = 0; i < maxSize; i += DATA_SIZE) {
    otaPrintln("Read Offset: 0x%4X", (gOtaImageInfoStart + i));
    sl_zigbee_af_eeprom_read((gOtaImageInfoStart + i), data, DATA_SIZE);
    sl_zigbee_af_print_cert(data);  // certs are 48 bytes long
    otaPrintFlush();
  }
}

static void printDataBlock(const uint8_t* block)
{
  uint8_t i;
  for (i = 0; i < DATA_SIZE; i += 8) {
    otaPrintFlush();
    otaPrintln("%X %X %X %X %X %X %X %X",
               block[i],
               block[i + 1],
               block[i + 2],
               block[i + 3],
               block[i + 4],
               block[i + 5],
               block[i + 6],
               block[i + 7]);
    otaPrintFlush();
  }
}

void sl_zigbee_af_eeprom_test(void)
{
  uint8_t data[DATA_SIZE];
  uint8_t i;
  uint32_t addressOffset = 0;
  uint8_t value;
  uint8_t length = 4;

  for (i = 0; i < 2; i++) {
    uint32_t address = addressOffset + (i * DATA_SIZE);
    value = 0x09 + i;
    memset(data, value, DATA_SIZE);
    otaPrintln("Writing value 0x%X to address 0x%4X", value, address);
    sl_zigbee_af_eeprom_write(address, data, DATA_SIZE);
    memset(data, 0, DATA_SIZE);
    sl_zigbee_af_eeprom_read(address, data, DATA_SIZE);
    printDataBlock(data);
    otaPrintln("");
    addressOffset += 240;  // this is less than the ATMEL part's page
                           // size (256) which means read/write operations
                           // will span two pages
  }

  addressOffset = 0;
  value = 0x02;
  otaPrintln("Re-writing value 0x%X of length %d to address 0x%4X",
             value,
             length,
             addressOffset);
  memset(data, value, DATA_SIZE);
  sl_zigbee_af_eeprom_write(addressOffset, data, length);
  memset(data, 0, DATA_SIZE);
  sl_zigbee_af_eeprom_read(addressOffset, data, DATA_SIZE);
  printDataBlock(data);
  otaPrintln("");
  //  writeInt32uToEeprom(value, addressOffset);
}

#endif // DEBUG_PRINT
