/*!
 * @file    quadruped_comm.h
 * @author  hanyuanqiang
 * @brief   Communication interface class, It can be realized by LCM DDS UDP
 *
 */
#ifndef BZL_QUADRUPED_SDK_QUADRUPED_COMM_H
#define BZL_QUADRUPED_SDK_QUADRUPED_COMM_H

#include "bzl_quadruped_sdk/bzl_quadruped_cmd.h"

namespace BZL_QUADRUPED_SDK
{
    class QuadrupedComm
    {

    public:

        QuadrupedComm() {}
        virtual ~QuadrupedComm() {}

        virtual void Init() =0;
        virtual void Send(const BodyCmd& bodyCmd) =0;
        virtual void Send(const GamepadCmd& gamepadCmd) =0;
        virtual void Send(const JointCmd& jointCmd) =0;
        virtual void Get(BodyState& bodyState) =0;
        virtual void Get(JointState& jointState) =0;
        virtual void Get(SensorState& sensorState) =0;
    };
}

#endif