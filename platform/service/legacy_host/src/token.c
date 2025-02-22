/***************************************************************************//**
 * @file
 * @brief
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
// For ftruncate(2) in glibc.
#define _XOPEN_SOURCE 700

#include PLATFORM_HEADER
#include CONFIGURATION_HEADER
#include "../inc/token.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <err.h>
#include <fcntl.h>
#include <sysexits.h>
#include <unistd.h>

// TODO: This implementation includes token-stack.h for access to token
// definitions.  In a host/NCP configuration, the tokens defined directly in
// token-stack.h are used by the NCP, not the host.  token-stack.h should be
// set up to exclude stack tokens when compiling a host application.  The
// inclusion of NCP tokens on the host is harmless, although it does waste disk
// space and is potentially confusing.

// TODO: This implementation treats manufacturing tokens as read-only
// application tokens.  Manufacturing tokens should be initialized to FF and be
// writable at runtime only if the value is FF.

// TODO: This implementation does not handle upgrading older token files.  This
// is not currently an issue because there is only one version of the token
// file.  Future versions, however, should recognize old token files and
// migrate them to the new format.  Instead, this condition is treated as an
// incompatible token file, which results in all values getting reset to the
// defaults.

// Version 1 format:
//   version:1 creator[0]:2 isCnt[0]:1 size[0]:1 arraySize[0]:1 data[0]:m_0 ...
//   creator[n]:2 isCnt[n]:1 size[n]:1 arraySize[n]:1 data[n]:m_n
// Version 2 format:
//   version:1 creator[0]:4 isCnt[0]:1 size[0]:1 arraySize[0]:1 data[0]:m_0 ...
//   creator[n]:2 isCnt[n]:1 size[n]:1 arraySize[n]:1 data[n]:m_n
#define VERSION 2

extern const uint32_t tokenNvm3Keys[];
extern const bool tokenIsCnt[];
extern const uint8_t tokenSize[];
extern const uint8_t tokenArraySize[];
extern const void * const tokenDefaults[];

// TODO: Don't include stack tokens on the host.
#define DEFINETOKENS
#define TOKEN_MFG TOKEN_DEF
#define TOKEN_DEF(name, creator, iscnt, isidx, type, arraysize, ...) \
  TOKEN_##name##_ADDRESS,
static const uint16_t addresses[] = {
    #include "stack/config/token-stack.h"
};
#undef TOKEN_DEF
#undef TOKEN_MFG
#undef DEFINETOKENS

static void initializeTokenSystem(void);
static void resetTokenData(void);
static size_t getNvmOffset(uint16_t token, uint8_t index, uint8_t len);
static void upgradeTokenFileToVersion2(int fd);

#ifndef SL_ZIGBEE_AF_TOKEN_FILENAME
    #define SL_ZIGBEE_AF_TOKEN_FILENAME "host_token.nvm"
#endif

// #define SL_ZIGBEE_AF_HOST_TOKEN_DEBUG
#ifdef SL_ZIGBEE_AF_HOST_TOKEN_DEBUG
  #define hostTokenDebugPrintf(...) fprintf(stdout, __VA_ARGS__)
#else
  #define hostTokenDebugPrintf(...)
#endif

// mmap(2) returns MAP_FAILED on failure, which is not necessarily defined the
// same as NULL.
static uint8_t *nvm = MAP_FAILED;
#define isInitialized() (nvm != MAP_FAILED)

#define PER_TOKEN_OVERHEAD  \
  (sizeof(tokenNvm3Keys[0]) \
   + sizeof(tokenIsCnt[0])  \
   + sizeof(tokenSize[0])   \
   + sizeof(tokenArraySize[0]))
#define TOTAL_SIZE                    \
  (1 /* version overhead */           \
   + TOKEN_COUNT * PER_TOKEN_OVERHEAD \
   + TOKEN_MAXIMUM_SIZE)

typedef struct {
  size_t offset; // offset from start of nvm file
  bool present;  // true if entry is present in nvm file
} nvmCreatorOffsetType;

// keeps track of token offsets in nvm file (helps with rearranged tokens)
// when populating, each index is maintained to be the same as the creator's
// index in tokenNvm3Keys[], tokenIsCnt[] etc.
static nvmCreatorOffsetType nvmCreatorOffset[TOKEN_COUNT];

void halInternalGetTokenData(void *data, uint16_t token, uint8_t index, uint8_t len)
{
  {
    DECLARE_INTERRUPT_STATE;
    DISABLE_INTERRUPTS();
    size_t offset = getNvmOffset(token, index, len);
    memcpy(data, nvm + offset, len);
    RESTORE_INTERRUPTS();
  }
}

void halInternalSetTokenData(uint16_t token, uint8_t index, void *data, uint8_t len)
{
  {
    DECLARE_INTERRUPT_STATE;
    DISABLE_INTERRUPTS();
    size_t offset = getNvmOffset(token, index, len);
    memcpy(nvm + offset, data, len);
    if (msync(nvm, TOKEN_MAXIMUM_SIZE, MS_SYNC) == -1) {
      err(EX_IOERR, "Could not write " SL_ZIGBEE_AF_TOKEN_FILENAME " to disk");
    }
    RESTORE_INTERRUPTS();
  }
}

void halInternalGetMfgTokenData(void *data, uint16_t token, uint8_t index, uint8_t len)
{
  // No manufacturing tokens on host.
  assert(false);
}

void halInternalSetMfgTokenData(uint16_t token, void *data, uint8_t len)
{
  // No manufacturing tokens on host.
  assert(false);
}

// check if token in nvm file is still present in stack/app
// if present, return new index in tokenNvm3Keys, else return false
static bool isOldToken(uint32_t tokCreator, size_t* index)
{
  for (size_t i = 0; i < TOKEN_COUNT; i++) {
    if (tokenNvm3Keys[i] == tokCreator) {
      *index = i;
      return true;
    }
  }
  return false;
}

// should we copy nvm values?
// if index is present (was populated) and token structure is same, return true
// else return false, so that we reset values
static bool copyNvm(uint8_t* nvmData,
                    size_t index,
                    size_t* tokOffset)
{
  uint8_t* nvmTokFinger;
  bool nvmTokIsCnt;
  size_t nvmTokSize, nvmTokArraySize;

  if (nvmCreatorOffset[index].present) {
    nvmTokFinger = nvmData + nvmCreatorOffset[index].offset;
    uint32_t tokCreator = (nvmTokFinger[0] << 24) + (nvmTokFinger[1] << 16) + (nvmTokFinger[2] << 8) + nvmTokFinger[3];
    assert(tokenNvm3Keys[index] == tokCreator);
    nvmTokIsCnt = nvmTokFinger[4];
    nvmTokSize = nvmTokFinger[5];
    nvmTokArraySize = nvmTokFinger[6];

    if (tokenIsCnt[index] == nvmTokIsCnt
        && tokenSize[index] == nvmTokSize) {
      if (tokenArraySize[index] < nvmTokArraySize) {
        hostTokenDebugPrintf("Changed Token - array size reduced: %4x\n", tokenNvm3Keys[index]);
      } else if (tokenArraySize[index] > nvmTokArraySize) {
        hostTokenDebugPrintf("Changed Token - array size increased: %4x\n", tokenNvm3Keys[index]);
      } else {
        hostTokenDebugPrintf("Unchanged Token: %4x\n", tokenNvm3Keys[index]);
      }
      *tokOffset = nvmCreatorOffset[index].offset;
      return true;
    } else { // reset resized token
      hostTokenDebugPrintf("Changed Token - reset: %4x\n", tokenNvm3Keys[index]);
      return false;
    }
  }

  hostTokenDebugPrintf("New Token: %4x\n", tokenNvm3Keys[index]);
  return false;
}

static void initializeTokenSystem(void)
{
  assert(!isInitialized());

  int fd = open(SL_ZIGBEE_AF_TOKEN_FILENAME,
                (O_RDWR | O_CREAT | O_CLOEXEC), // open read/write, create if missing, close on exit
                (S_IRUSR | S_IWUSR));    // read/write permissions for owner
  if (fd == -1) {
    err(EX_NOINPUT, "Could not open or create " SL_ZIGBEE_AF_TOKEN_FILENAME);
  }

  struct stat buf;
  if (fstat(fd, &buf) == -1) {
    err(EX_IOERR, "Could not determine size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
  }

  bool reset = (buf.st_size == 0); // new or empty file
  if (reset && ftruncate(fd, TOTAL_SIZE) == -1) {
    err(EX_IOERR, "Could not set size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
  }

  nvm = mmap(NULL,                     // let system choose address
             (buf.st_size == 0 ? TOTAL_SIZE : (size_t) buf.st_size),
             (PROT_READ | PROT_WRITE), // data can be read/written
             MAP_SHARED,               // writes change the file
             fd,
             0);                       // no offset
  if (!isInitialized()) {
    err(EX_UNAVAILABLE, "Could not map " SL_ZIGBEE_AF_TOKEN_FILENAME " to memory");
  }

  // TODO: Handle older token files.
  if (!reset) {
    if (*nvm == 1 && VERSION == 2) {
      // upgrade token file from version 1 to version 2
      upgradeTokenFileToVersion2(fd);
      // we upgraded the token file then need to refresh its metadata info
      if (fstat(fd, &buf) == -1) {
        err(EX_IOERR, "Could not determine size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
      }
    }
    reset = (*nvm != VERSION);
  }

  if (!reset) {
    // save original content as we keep modifying "nvm"
    uint8_t *origNvm = (uint8_t*)malloc(buf.st_size * sizeof(uint8_t));
    memcpy(origNvm, nvm, buf.st_size);

    uint8_t *finger = origNvm + 1; // skip version; already verified

    // read token file and save old token offsets (helps with rearranged tokens)
    while ((finger - origNvm) < buf.st_size) { // iterate through origNvm
      uint32_t tokCreator = (finger[0] << 24) + (finger[1] << 16) + (finger[2] << 8) + finger[3];
      uint8_t tokSize = finger[5];
      uint8_t tokArraySize = finger[6];
      size_t newIndex;
      if (isOldToken(tokCreator, &newIndex)) { // if token is old, save its offset in new index
        assert(!nvmCreatorOffset[newIndex].present); // should not have already been set
        nvmCreatorOffset[newIndex].offset = (finger - origNvm);
        nvmCreatorOffset[newIndex].present = true;
      } else { // token is removed so ignore it
        hostTokenDebugPrintf("Removed Token: %4x\n", tokCreator);
      }
      finger += PER_TOKEN_OVERHEAD + (tokSize * tokArraySize);
    }

    assert((finger - origNvm) == buf.st_size); // corrupt nvm file check

    if (ftruncate(fd, TOTAL_SIZE) == -1) {
      err(EX_IOERR, "Could not set size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
    }

    // rewrite tokens file
    finger = nvm + 1; // skip version; already verified
    size_t i, j, nvmArraySizeToCopy, nvmTokOffset, nvmTokArraySize;
    uint8_t* nvmTokFinger;
    for (i = 0; i < TOKEN_COUNT; i++) { // iterate through stack/app tokens
      *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0xFF000000) >> 24);
      *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0x00FF0000) >> 16);
      *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0x0000FF00) >> 8);
      *finger++ = (uint8_t)(tokenNvm3Keys[i] & 0x000000FF);
      *finger++ = tokenIsCnt[i];
      *finger++ = tokenSize[i];
      *finger++ = tokenArraySize[i];
      nvmArraySizeToCopy = 0;

      if (copyNvm(origNvm, i, &nvmTokOffset)) { // if true, get token offset from nvm
        nvmTokFinger = origNvm + nvmTokOffset;
        nvmTokArraySize = nvmTokFinger[4 + 1 + 1];
        nvmArraySizeToCopy = nvmTokArraySize < tokenArraySize[i] ? nvmTokArraySize : tokenArraySize[i];
        nvmTokFinger += PER_TOKEN_OVERHEAD;
      }

      // copy as many array elements as possible
      memcpy(finger,
             nvmTokFinger,
             nvmArraySizeToCopy * tokenSize[i]);

      finger += nvmArraySizeToCopy * tokenSize[i];

      // set the remaining array elements (if any) to default
      for (j = nvmArraySizeToCopy; j < tokenArraySize[i]; j++) {
        memcpy(finger,
               (uint8_t *)tokenDefaults[i],
               tokenSize[i]);
        finger += tokenSize[i];
      }
    }
    free(origNvm);
    origNvm = NULL;
  }

  if (reset) {
    resetTokenData();
  }
}

static void resetTokenData(void)
{
  assert(isInitialized());

  uint8_t *finger = nvm;
  *finger++ = VERSION;
  for (size_t i = 0; i < TOKEN_COUNT; i++) {
    *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0xFF000000) >> 24);
    *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0x00FF0000) >> 16);
    *finger++ = (uint8_t)((tokenNvm3Keys[i] & 0x0000FF00) >> 8);
    *finger++ = (uint8_t)(tokenNvm3Keys[i] & 0x000000FF);
    *finger++ = tokenIsCnt[i];
    *finger++ = tokenSize[i];
    *finger++ = tokenArraySize[i];
    finger += tokenArraySize[i] * tokenSize[i];
  }

  #define DEFINETOKENS
  #define TOKEN_MFG(name, creator, iscnt, isidx, type, arraysize, ...)
  #define TOKEN_DEF(name, creator, iscnt, isidx, type, arraysize, ...)     \
  {                                                                        \
    type data = __VA_ARGS__;                                               \
    if (arraysize == 1) {                                                  \
      halInternalSetTokenData(TOKEN_##name, 0x7F, &data, sizeof(type));    \
    } else {                                                               \
      uint8_t i;                                                           \
      for (i = 1; i <= arraysize; i++) {                                   \
        halInternalSetTokenData(TOKEN_##name, i - 1, &data, sizeof(type)); \
      }                                                                    \
    }                                                                      \
  }
  #include "stack/config/token-stack.h"
  #undef TOKEN_DEF
  #undef TOKEN_MFG
  #undef DEFINETOKENS
}

static size_t getNvmOffset(uint16_t token, uint8_t index, uint8_t len)
{
  if (!isInitialized()) {
    initializeTokenSystem();
    assert(isInitialized());
  }

  size_t offset = 1 + addresses[token] + (token + 1) * PER_TOKEN_OVERHEAD;
  if (index != 0x7F) {
    offset += (index * len);
  }
  return offset;
}

static void upgradeTokenFileToVersion2(int fd)
{
  struct stat buf;
  if (fstat(fd, &buf) == -1) {
    err(EX_IOERR, "Could not determine size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
  }

  // backup version 1 token file so we can use that to copy data to version 2 token file
  uint8_t *oldTokenFileBuf = (uint8_t*)malloc(buf.st_size * sizeof(uint8_t));
  memcpy(oldTokenFileBuf, nvm, buf.st_size);

  if (ftruncate(fd, TOTAL_SIZE) == -1) {
    err(EX_IOERR, "Could not set size of " SL_ZIGBEE_AF_TOKEN_FILENAME);
  }

  // initialize new version 2 token file
  resetTokenData();

  uint8_t *newTokenFileFinger = nvm + 1; // skip version
  const uint8_t *oldTokenFileFinger = oldTokenFileBuf + 1; // skip version

  for (size_t token = 0; token < TOKEN_COUNT; token++) {
    size_t nByteToCopy = (tokenSize[token] * tokenArraySize[token]);
    newTokenFileFinger += PER_TOKEN_OVERHEAD;
    oldTokenFileFinger += (PER_TOKEN_OVERHEAD - 2); // version 1 overhead is 2 byte less than version 2 overhead
    assert(oldTokenFileFinger < (oldTokenFileBuf + buf.st_size));
    memcpy(newTokenFileFinger, oldTokenFileFinger, nByteToCopy);
    newTokenFileFinger += nByteToCopy;
    oldTokenFileFinger += nByteToCopy;
  }

  if (msync(nvm, TOTAL_SIZE, MS_SYNC) == -1) {
    err(EX_IOERR, "Could not write " SL_ZIGBEE_AF_TOKEN_FILENAME " to disk");
  }
  free(oldTokenFileBuf);
  oldTokenFileBuf = NULL;
}
