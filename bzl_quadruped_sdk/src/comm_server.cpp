/*!
 * @file    comm_server.cpp
 * @author  hanyuanqiang
 * @brief   Implementation of server communication using LCM
 *
 */

#include <iostream>
#include <unistd.h>

#include "comm_server.h"

using namespace BZL_QUADRUPED_SDK;

CommServer::CommServer(ControlFSMData<float>& data)
    : _commServerLcm("udpm://239.255.76.67:7667?ttl=1"),
      _pFSMData(&data),
      _logger("bzl_quadruped_sdk") {

    memset(&_bodyCmd, 0, sizeof(_bodyCmd));
    memset(&_jointCmd, 0, sizeof(_jointCmd));
    memset(&_bodyState, 0, sizeof(_bodyState));
    
    pthread_mutex_init(&_bodyCmdMutex, NULL);
    pthread_mutex_init(&_jointCmdMutex, NULL);
    pthread_mutex_init(&_bodyStateMutex, NULL);

    if (!_commServerLcm.good())
    {
        QUADRUPED_ERROR(_logger, "init lcm failed!");
    }

    _commServerLcm.subscribe(strBodyCmdChannel, &CommServer::HandleBodyCmd, this);
    _commServerLcm.subscribe(strJointCmdChannel, &CommServer::HandleJointCmd, this);

    _commServerLcmThread = std::thread(&CommServer::HandleLCM, this);

    _pBodyStateTask = std::make_shared<PeriodicMemberFunction<CommServer>> (
      &_taskManager, _bodyStateTaskPeriod, "bodyStateTask", &CommServer::BodyStateTask, this);
    _pBodyStateTask->start();
}

CommServer::~CommServer() {
    _commServerLcmThread.join();
}

void CommServer::HandleLCM() {
    while (1)
    {
        _commServerLcm.handle();
    }
}

void CommServer::Run() {

    _bodyState.mode = (RC_mode)(_pFSMData->_desiredStateCommand->rcCommand->mode);
    _bodyState.forwardSpeed = _pFSMData->_stateEstimator->getResult().vBody[0];
    _bodyState.sideSpeed = _pFSMData->_stateEstimator->getResult().vBody[0];
    _bodyState.rotateSpeed = _pFSMData->_stateEstimator->getResult().omegaBody[2];
    _bodyState.bodyHeight = _pFSMData->_stateEstimator->getResult().position[2];
    _bodyState.forwardPosition = _pFSMData->_stateEstimator->getResult().position[0];
    _bodyState.sidePosition = _pFSMData->_stateEstimator->getResult().position[1];

    if (_isJointCmd)
    {
        // TODO(hanyuanqiang):
    }
    else if (_isBodyCmd)
    {
        _pFSMData->_desiredStateCommand->rcCommand->mode = (double)_bodyCmd.mode;
        _pFSMData->_desiredStateCommand->leftAnalogStick[1] = _bodyCmd.forwardSpeed;
        _pFSMData->_desiredStateCommand->leftAnalogStick[0] = _bodyCmd.sideSpeed;
        _pFSMData->_desiredStateCommand->rightAnalogStick[0] = _bodyCmd.rotateSpeed;
    }

}

void CommServer::HandleBodyCmd(
    const lcm::ReceiveBuffer* pBuf,
    const std::string& str,
    const body_cmd_lcmt* pMsg) {
    
    (void)pBuf;
    (void)str;

    pthread_mutex_lock(&_bodyCmdMutex);
    if (sizeof(body_cmd_lcmt) == sizeof(BodyCmd))
    {
        _isBodyCmd = true;
        memcpy(&_bodyCmd, pMsg, sizeof(body_cmd_lcmt));
    }
    else
    {
        QUADRUPED_ERROR(_logger, "BodyCmd!");
    }
    pthread_mutex_unlock(&_bodyCmdMutex);
}

void CommServer::HandleJointCmd (
    const lcm::ReceiveBuffer* pBuf,
    const std::string& str,
    const spi_command_t* pMsg) {

    (void)pBuf;
    (void)str;

    pthread_mutex_lock(&_jointCmdMutex);
    if (sizeof(spi_command_t) == sizeof(JointCmd))
    {
        _isJointCmd = true;
        memcpy(&_jointCmd, pMsg, sizeof(spi_command_t));
    }
    else
    {
        QUADRUPED_ERROR(_logger, "HandleJointCmd!");
    }
    pthread_mutex_unlock(&_jointCmdMutex);
}

void CommServer::BodyStateTask() {

    body_state_lcmt bodyStateLcmt;

    if (sizeof(body_state_lcmt) == sizeof(BodyState))
    {
        pthread_mutex_lock(&_bodyStateMutex);
        memcpy(&bodyStateLcmt, &_bodyState, sizeof(body_state_lcmt));
        pthread_mutex_unlock(&_bodyStateMutex);

        _commServerLcm.publish(strBodyStateChannel, &bodyStateLcmt);
    }
    else
    {
        QUADRUPED_ERROR(_logger, "BodyStateTask!");
    }
}
