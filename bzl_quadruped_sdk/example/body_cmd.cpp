#include <bzl_quadruped_sdk/bzl_quadruped.h>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace BZL_QUADRUPED_SDK;

#define USE_SHUTDOWN

const float gSpeed = 0.1;
const float gMovePosition = 0.6;

int main(void)
{
    int8_t states = 0;
    std::shared_ptr<BzlQuadruped> pBzlQuadruped = std::make_shared<BzlQuadruped>();

    BodyCmd bodyCmd;
    BodyState bodyState;

    memset(&bodyCmd, 0, sizeof(bodyCmd));
    memset(&bodyState, 0, sizeof(bodyState));

    bodyCmd.mode = RC_mode::STAND;              // 站立
    pBzlQuadruped->SendCmd(bodyCmd);
    sleep(5);

    bodyCmd.mode = RC_mode::LOCOMOTION;         // 踏步
    pBzlQuadruped->SendCmd(bodyCmd);
    sleep(3);

    while (1)
    {
        bodyCmd.mode = RC_mode::LOCOMOTION;
        bodyCmd.forwardSpeed = 0;
        bodyCmd.sideSpeed = 0;
        pBzlQuadruped->GetState(bodyState);

        switch (states)
        {
        case 0:
            (bodyState.forwardPosition <= gMovePosition) ?
            (bodyCmd.forwardSpeed = gSpeed) :       // 前进
            (states = 1);
        break;
        case 1:
            (bodyState.sidePosition <= gMovePosition) ?
            (bodyCmd.sideSpeed = gSpeed) :          // 左移
            (states = 2);
        break;
        case 2:
            (bodyState.forwardPosition >= 0.0) ?
            (bodyCmd.forwardSpeed = -gSpeed) :      // 后退
            (states = 3);
        break;
        case 3:
            (bodyState.sidePosition >= 0.0) ?
            (bodyCmd.sideSpeed = -gSpeed) :         // 右移
            (states = 4);
        break;
        }

        if (states == 4)
        {
            break;
        }

        pBzlQuadruped->SendCmd(bodyCmd);
    }

    bodyCmd.mode = RC_mode::STAND;              // 站立
    pBzlQuadruped->SendCmd(bodyCmd);
    sleep(5);

    bodyCmd.mode = RC_mode::PRONE;              // 趴下
    pBzlQuadruped->SendCmd(bodyCmd);
    sleep(6);

#ifdef USE_SHUTDOWN
    pBzlQuadruped->ShutDown();                  // 禁能后，禁能后机器需要重启
#endif

    return 0;
}