/*!
 * @file    bzl_quadruped.h
 * @author  hanyuanqiang
 * @brief   BZL quadruped API entry, Contains the definition of command and state
 *
 */
#ifndef BZL_QUADRUPED_SDK_BZL_QUADRUPED_H
#define BZL_QUADRUPED_SDK_BZL_QUADRUPED_H

#include <bzl_quadruped_sdk/bzl_quadruped_cmd.h>

namespace BZL_QUADRUPED_SDK
{
    class BzlQuadruped
    {
    public:
        BzlQuadruped();
        ~BzlQuadruped();
        bool SendCmd(const BodyCmd& bodyCmd);
        bool SendCmd(const GamepadCmd& gamepadCmd);
        bool SendCmd(const JointCmd& jointCmd);
        bool ShutDown(void);                // 注意非紧急情况或者趴下状态下，不可调用该函数，调用该函数后，机器需要重启才能继续工作
        bool GetState(BodyState& bodyState);
        bool GetState(JointState& jointState);
        bool GetState(SensorState& sensorState);
    };   
}

#endif