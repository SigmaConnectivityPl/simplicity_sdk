/*******************************************************************************
 * @file
 * @brief Definitions for a CPC based NCP interface to the OpenThread stack
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

#include "ncp_cpc.hpp"

#include <stdio.h>

#include <openthread/ncp.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/misc.h>

#include "openthread-core-config.h"
#include "openthread-system.h" // for otSysEventSignalPending()
#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/new.hpp"
#include "instance/instance.hpp"

#if OPENTHREAD_CONFIG_NCP_CPC_ENABLE

namespace ot {
namespace Ncp {

static OT_DEFINE_ALIGNED_VAR(sNcpRaw, sizeof(NcpCPC), uint64_t);

#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && OPENTHREAD_RADIO
extern "C" void otAppNcpInitMulti(otInstance **aInstances, uint8_t aCount)
{
    NcpCPC       *ncpCPC = nullptr;
    ot::Instance *instances[SPINEL_HEADER_IID_MAX];

    OT_ASSERT(aCount < SPINEL_HEADER_IID_MAX + 1);
    OT_ASSERT(aCount > 0);
    OT_ASSERT(aInstances != nullptr);
    OT_ASSERT(aInstances[0] != nullptr);

    for (int i = 0; i < aCount; i++)
    {
        instances[i] = static_cast<ot::Instance *>(aInstances[i]);
    }

    ncpCPC = new (&sNcpRaw) NcpCPC(instances, aCount);

    if (ncpCPC == nullptr || ncpCPC != NcpBase::GetNcpInstance())
    {
        OT_ASSERT(false);
    }
}
#endif

extern "C" void otAppNcpInit(otInstance *aInstance)
{
    NcpCPC   *ncpCPC   = nullptr;
    Instance *instance = static_cast<Instance *>(aInstance);

    ncpCPC = new (&sNcpRaw) NcpCPC(instance);

    if (ncpCPC == nullptr || ncpCPC != NcpBase::GetNcpInstance())
    {
        OT_ASSERT(false);
    }
}

#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && OPENTHREAD_RADIO
NcpCPC::NcpCPC(Instance **aInstances, uint8_t aCount)
    : NcpBase(aInstances, aCount)
    , mIsReady(false)
    , mIsWriting(false)
    , mCpcSendTask(*aInstances[0], SendToCPC)
    , mCpcEndpointErrorTask(*aInstances[0], HandleEndpointError)
    , mCpcOpenEndpointTask(*aInstances[0], HandleOpenEndpoint)
{
}
#endif

NcpCPC::NcpCPC(Instance *aInstance)
    : NcpBase(aInstance)
    , mIsReady(false)
    , mIsWriting(false)
    , mCpcSendTask(*aInstance, SendToCPC)
    , mCpcEndpointErrorTask(*aInstance, HandleEndpointError)
    , mCpcOpenEndpointTask(*aInstance, HandleOpenEndpoint)
{
}

void NcpCPC::HandleOpenEndpoint(Tasklet &aTasklet)
{
    OT_UNUSED_VARIABLE(aTasklet);
    static_cast<NcpCPC *>(GetNcpInstance())->HandleOpenEndpoint();
}

void NcpCPC::HandleOpenEndpoint(void)
{
    sl_status_t status = sli_cpc_open_service_endpoint(&mUserEp, SL_CPC_ENDPOINT_15_4, 0, 1);

    if (status == SL_STATUS_ALREADY_EXISTS)
    {
        return;
    }
    else if (status == SL_STATUS_BUSY)
    {
        static_cast<NcpCPC *>(GetNcpInstance())->mCpcOpenEndpointTask.Post();
        return;
    }

    OT_ASSERT(status == SL_STATUS_OK);

    status = sl_cpc_set_endpoint_option(&mUserEp,
                                        SL_CPC_ENDPOINT_ON_IFRAME_WRITE_COMPLETED,
                                        reinterpret_cast<void *>(HandleCPCSendDone));

    OT_ASSERT(status == SL_STATUS_OK);

    status = sl_cpc_set_endpoint_option(&mUserEp,
                                        SL_CPC_ENDPOINT_ON_IFRAME_RECEIVE,
                                        reinterpret_cast<void *>(HandleCPCReceive));

    OT_ASSERT(status == SL_STATUS_OK);

    status = sl_cpc_set_endpoint_option(&mUserEp,
                                        SL_CPC_ENDPOINT_ON_ERROR,
                                        reinterpret_cast<void *>(HandleCPCEndpointError));

    OT_ASSERT(status == SL_STATUS_OK);

    mTxFrameBuffer.SetFrameAddedCallback(HandleFrameAddedToNcpBuffer, this);
}

void NcpCPC::HandleFrameAddedToNcpBuffer(void                    *aContext,
                                         Spinel::Buffer::FrameTag aTag,
                                         Spinel::Buffer::Priority aPriority,
                                         Spinel::Buffer          *aBuffer)
{
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aTag);
    OT_UNUSED_VARIABLE(aPriority);

    static_cast<NcpCPC *>(aContext)->HandleFrameAddedToNcpBuffer();
}

void NcpCPC::HandleFrameAddedToNcpBuffer(void)
{
    if (mIsReady && !mIsWriting)
        mCpcSendTask.Post();
}

void NcpCPC::SendToCPC(Tasklet &aTasklet)
{
    OT_UNUSED_VARIABLE(aTasklet);
    static_cast<NcpCPC *>(GetNcpInstance())->SendToCPC();
}

// may need to be updated to handle sleepy devices. Refer to NcpUart::EncodeAndSendToUart
void NcpCPC::SendToCPC(void)
{
    Spinel::Buffer &txFrameBuffer = mTxFrameBuffer;
    uint16_t        bufferLen;
    uint16_t        offset = 0;

    VerifyOrExit(mIsReady && !mIsWriting && !txFrameBuffer.IsEmpty());

    mIsWriting = true;

    // Concatenate multiple spinel buffers for efficiency over CPC.
    while (!txFrameBuffer.IsEmpty())
    {
        IgnoreError(txFrameBuffer.OutFrameBegin());
        bufferLen = txFrameBuffer.OutFrameGetLength();
        if (offset + sizeof(uint16_t) + bufferLen < kCpcTxBufferSize)
        {
            BigEndian::WriteUint16(bufferLen, mCpcTxBuffer + offset);
            offset += sizeof(uint16_t);
            txFrameBuffer.OutFrameRead(bufferLen, mCpcTxBuffer + offset);
            offset += bufferLen;
            IgnoreError(txFrameBuffer.OutFrameRemove());
        }
        else
        {
            break;
        }
    }

    if (sl_cpc_write(&mUserEp, mCpcTxBuffer, offset, 0, NULL) != SL_STATUS_OK)
    {
        mIsWriting = false;
    }

exit:
    // If the CPCd link isn't ready yet, just remove the frame from
    // the queue so that it doesn't fill up unnecessarily
    if (!mIsReady)
    {
        IgnoreError(txFrameBuffer.OutFrameRemove());
    }

    return;
}

void NcpCPC::HandleCPCSendDone(sl_cpc_user_endpoint_id_t endpoint_id, void *buffer, void *arg, sl_status_t status)
{
    OT_UNUSED_VARIABLE(endpoint_id);
    OT_UNUSED_VARIABLE(buffer);
    OT_UNUSED_VARIABLE(arg);
    OT_UNUSED_VARIABLE(status);

    static_cast<NcpCPC *>(GetNcpInstance())->HandleSendDone();
}

void NcpCPC::HandleSendDone(void)
{
    mIsWriting = false;
    memset(mCpcTxBuffer, 0, sizeof(mCpcTxBuffer));
}

void NcpCPC::HandleCPCReceive(sl_cpc_user_endpoint_id_t endpoint_id, void *arg)
{
    OT_UNUSED_VARIABLE(endpoint_id);
    OT_UNUSED_VARIABLE(arg);
    otSysEventSignalPending(); // wakeup ot task
}

void NcpCPC::HandleCPCEndpointError(uint8_t endpoint_id, void *arg)
{
    OT_UNUSED_VARIABLE(endpoint_id);
    OT_UNUSED_VARIABLE(arg);

    // Can't close and open endpoints in this context
    static_cast<NcpCPC *>(GetNcpInstance())->mCpcEndpointErrorTask.Post();
}

void NcpCPC::HandleEndpointError(Tasklet &aTasklet)
{
    OT_UNUSED_VARIABLE(aTasklet);
    static_cast<NcpCPC *>(GetNcpInstance())->HandleEndpointError();
}

void NcpCPC::HandleEndpointError(void)
{
    OT_ASSERT(sl_cpc_close_endpoint(&mUserEp) == SL_STATUS_OK);
    mIsReady = false;
}

extern "C" void efr32CpcProcess(void)
{
    NcpCPC *ncpCPC = static_cast<NcpCPC *>(NcpBase::GetNcpInstance());

    if (ncpCPC != nullptr)
    {
        ncpCPC->ProcessCpc();
    }
}

void NcpCPC::ProcessCpc(void)
{
    sl_status_t status;
    void       *data;
    uint16_t    dataLength;

    HandleOpenEndpoint();

    status = sl_cpc_read(&mUserEp,
                         &data,
                         &dataLength,
                         0,
                         SL_CPC_FLAG_NO_BLOCK); // In bare-metal read is always
                                                // non-blocking, but with rtos
                                                // since this function is called
                                                // in the cpc task, it must not
                                                // block.

    SuccessOrExit(status);

    if (!mIsReady)
    {
        mIsReady = true;
    }

    super_t::HandleReceive(static_cast<uint8_t *>(data), dataLength);

    status = sl_cpc_free_rx_buffer(data);
    OT_ASSERT(status == SL_STATUS_OK);

exit:
    if (mIsReady && !mTxFrameBuffer.IsEmpty())
        mCpcSendTask.Post();
}

} // namespace Ncp
} // namespace ot

#endif // OPENTHREAD_CONFIG_NCP_CPC_ENABLE
