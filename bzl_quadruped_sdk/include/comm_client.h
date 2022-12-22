/*!
 * @file    comm_client.h
 * @author  hanyuanqiang
 * @brief   Implementation of client communication using LCM
 *
 */
#ifndef BZL_QUADRUPED_SDK_COMM_CLIENT_H
#define BZL_QUADRUPED_SDK_COMM_CLIENT_H

#include <thread>
#include <lcm-cpp.hpp>

#include "bzl_quadruped_sdk/bzl_quadruped_cmd.h"
#include "quadruped_comm.h"

#include "body_cmd_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "spi_command_t.hpp"
#include "body_state_lcmt.hpp"
#include "spi_data_t.hpp"
#include "microstrain_lcmt.hpp"

namespace BZL_QUADRUPED_SDK
{
    constexpr char strBodyCmdChannel[] =    "body_cmd";
    constexpr char strGamepadCmdChannel[] = "interface";
    constexpr char strJointCmdChannel[] =   "joint_cmd";
    constexpr char strBodyStateChannel[] =  "body_state";
    constexpr char strJointStateChannel[] = "spi_data";
    // constexpr char strSensorStateChannel[] = "LcmSensorState";
    constexpr char strSensorImuChannel[] = "microstrain";

    class CommClient : public QuadrupedComm
    {

    public:
        CommClient();
        virtual ~CommClient();

        virtual void Init();
        virtual void Send(const BodyCmd& bodyCmd);
        virtual void Send(const GamepadCmd& gamepadCmd);
        virtual void Send(const JointCmd& jointCmd);
        virtual void Get(BodyState& bodyState);
        virtual void Get(JointState& jointState);
        virtual void Get(SensorState& sensorState);

    private:
        lcm::LCM _commClientLcm;
        std::thread _commClientLcmThread;

        body_cmd_lcmt _bodyCmd;
        gamepad_lcmt _gamepadCmd;
        spi_command_t _jointCmd;
        BodyState _bodyState;
        JointState _jointState;
        SensorState _sensorState;

        pthread_mutex_t _bodyCmdMutex;
        pthread_mutex_t _gamepadCmdMutex;
        pthread_mutex_t _jointCmdMutex;
        pthread_mutex_t _bodyStateMutex;
        pthread_mutex_t _jointStateMutex;
        pthread_mutex_t _sensorStateMutex;
        
        void HandleBodyState(const lcm::ReceiveBuffer* pBuf, const std::string& str, const body_state_lcmt* pMsg);
        void HandleJointState(const lcm::ReceiveBuffer* pBuf, const std::string& str, const spi_data_t* pMsg);
        void HandleSensorImu(const lcm::ReceiveBuffer* pBuf, const std::string& str, const microstrain_lcmt* pMsg);
        // TODO(hanyuanqiang)：添加足端传感器、电量相关的信息Handle
        void HandleLCM();

    };

}

#endif