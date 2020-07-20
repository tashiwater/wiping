/**
 * @file  MasterControllerClientMock.h
 * @brief Master controller client mock class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MASTER_CONTROLLER_CLIENT_MOCK_H__
#define __MASTER_CONTROLLER_CLIENT_MOCK_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "MasterControllerClient/MasterControllerClient.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class MasterControllerClientMock : public MasterControllerClient
{
public:
    MasterControllerClientMock(int jointsNum);
    virtual ~MasterControllerClientMock();

    int32_t ReceiveStatus() override;
    void SendPacketInBuffer() override;

    void SetRecvBuffer(RecvPacket packet);
};

#endif
