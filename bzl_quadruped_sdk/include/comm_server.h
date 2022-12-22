/*!
 * @file    comm_server.h
 * @author  hanyuanqiang
 * @brief   Implementation of server communication using LCM
 *
 */
#ifndef BZL_QUADRUPED_SDK_COMM_SERVER_H
#define BZL_QUADRUPED_SDK_COMM_SERVER_H

#include <thread>
#include <lcm-cpp.hpp>

#include "bzl_quadruped_sdk/bzl_quadruped_cmd.h"
#include "quadruped_comm.h"
#include "Utilities/PeriodicTask.h"
#include "FSM_States/ControlFSMData.h"
#include "Logger/Logger.h"

#include "body_cmd_lcmt.hpp"
#include "spi_command_t.hpp"
#include "body_state_lcmt.hpp"

namespace BZL_QUADRUPED_SDK
{
    constexpr char strBodyCmdChannel[] =    "body_cmd";
    constexpr char strJointCmdChannel[] =   "joint_cmd";
    constexpr char strBodyStateChannel[] =  "body_state";

    class CommServer
    {
    public:
        CommServer(ControlFSMData<float>& data);
        ~CommServer();

        void Run();

    private:
        lcm::LCM _commServerLcm;
        std::thread _commServerLcmThread;
        PeriodicTaskManager _taskManager;
        ControlFSMData<float>* _pFSMData;

        BodyCmd _bodyCmd;
        JointCmd _jointCmd;
        BodyState _bodyState;
        bool _isBodyCmd = false;
        bool _isJointCmd = false;
        pthread_mutex_t _bodyCmdMutex;
        pthread_mutex_t _jointCmdMutex;
        pthread_mutex_t _bodyStateMutex;

        std::shared_ptr<PeriodicMemberFunction<CommServer>> _pBodyStateTask;
        float _bodyStateTaskPeriod = 0.010;  // 10ms
        BZL_QUADRUPED::Logger _logger;

        void HandleLCM();
        void HandleBodyCmd(const lcm::ReceiveBuffer* pBuf, const std::string& str, const body_cmd_lcmt* pMsg);
        void HandleJointCmd(const lcm::ReceiveBuffer* pBuf, const std::string& str, const spi_command_t* pMsg);
        void BodyStateTask();
    };
}

#endif