/*!
 * @file    comm_client.cpp
 * @author  hanyuanqiang
 * @brief   Implementation of client communication using LCM
 *
 */
#include <iostream>

#include "comm_client.h"
#include <unistd.h>

using namespace BZL_QUADRUPED_SDK;

CommClient::CommClient()
    : _commClientLcm("udpm://239.255.76.67:7667?ttl=255") {
    memset(&_gamepadCmd, 0, sizeof(_gamepadCmd));
    memset(&_bodyState, 0, sizeof(_bodyState));
    memset(&_jointState, 0, sizeof(_jointState));
    memset(&_sensorState, 0, sizeof(_sensorState));

    pthread_mutex_init(&_bodyCmdMutex, NULL);
    pthread_mutex_init(&_gamepadCmdMutex, NULL);
    pthread_mutex_init(&_jointCmdMutex, NULL);
    pthread_mutex_init(&_bodyStateMutex, NULL);
    pthread_mutex_init(&_jointStateMutex, NULL);
    pthread_mutex_init(&_sensorStateMutex, NULL);
}

CommClient::~CommClient(){
    _commClientLcmThread.join();

}

void CommClient::Init(){
    uint32_t retryTimes = 0;

    while (!_commClientLcm.good())
    {
        retryTimes++;
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "init lcm failed, retry times: " << retryTimes << std::endl;
        _commClientLcm = lcm::LCM("udpm://239.255.76.67:7667?ttl=255");
        sleep(1);
    }
    std::cout << "[OK] bzl_quadruped_sdk: "
                  << "init lcm succeed!" << std::endl;

    _commClientLcm.subscribe(strBodyStateChannel, &CommClient::HandleBodyState, this);
    _commClientLcm.subscribe(strJointStateChannel, &CommClient::HandleJointState, this);
    _commClientLcm.subscribe(strSensorImuChannel, &CommClient::HandleSensorImu, this);

    _commClientLcmThread = std::thread(&CommClient::HandleLCM, this);
}

void CommClient::Send(const BodyCmd& bodyCmd){
    pthread_mutex_lock(&_bodyCmdMutex);
    if (sizeof(BodyCmd) == sizeof(body_cmd_lcmt))
    {
        memcpy(&_bodyCmd, &bodyCmd, sizeof(body_cmd_lcmt));
        _commClientLcm.publish(strBodyCmdChannel, &_bodyCmd);
    }
    else
    {
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "Send BodyCmd!" << std::endl;
    }
    pthread_mutex_unlock(&_bodyCmdMutex);
}

void CommClient::Send(const GamepadCmd& gamepadCmd){
    pthread_mutex_lock(&_gamepadCmdMutex);
    if (sizeof(GamepadCmd) == sizeof(gamepad_lcmt))
    {
        memcpy(&_gamepadCmd, &gamepadCmd, sizeof(gamepad_lcmt));
        _commClientLcm.publish(strGamepadCmdChannel, &_gamepadCmd);
    }
    else
    {
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "Send GamepadCmd!" << std::endl;
    }  
    pthread_mutex_unlock(&_gamepadCmdMutex);
}

void CommClient::Send(const JointCmd& jointCmd){
    pthread_mutex_lock(&_jointCmdMutex);
    if (sizeof(JointCmd) == sizeof(spi_command_t))
    {
        memcpy(&_jointCmd, &jointCmd, sizeof(spi_command_t));
        _commClientLcm.publish(strJointCmdChannel, &_jointCmd);
    }
    else
    {
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "Send JointCmd!" << std::endl;
    }
    pthread_mutex_unlock(&_jointCmdMutex);
}

void CommClient::Get(BodyState& bodyState){
    pthread_mutex_lock(&_bodyStateMutex);
    bodyState = _bodyState;
    pthread_mutex_unlock(&_bodyStateMutex);
}


void CommClient::Get(JointState& jointState){
    pthread_mutex_lock(&_jointStateMutex);
    jointState = _jointState;
    pthread_mutex_unlock(&_jointStateMutex);
}

void CommClient::Get(SensorState& sensorState){
    pthread_mutex_lock(&_sensorStateMutex);
    sensorState = _sensorState;
    pthread_mutex_unlock(&_sensorStateMutex);
}

void CommClient::HandleLCM(){
    while (1)
    {
        _commClientLcm.handle();
    }
}

void CommClient::HandleBodyState(
    const lcm::ReceiveBuffer* pBuf,
    const std::string& str,
    const body_state_lcmt* pMsg) {
    
    (void)pBuf;
    (void)str;
    pthread_mutex_lock(&_bodyStateMutex);
    if (sizeof(body_state_lcmt) == sizeof(BodyState))
    {
        memcpy(&_bodyState, pMsg, sizeof(body_state_lcmt));
    }
    else
    {
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "BodyState!" << std::endl;
    }
    pthread_mutex_unlock(&_bodyStateMutex);
}

void CommClient::HandleJointState(
    const lcm::ReceiveBuffer* pBuf,
    const std::string& str,
    const spi_data_t* pMsg) {

    (void)pBuf;
    (void)str;
    pthread_mutex_lock(&_jointStateMutex);
    if (sizeof(spi_data_t) == sizeof(JointState))
    {
        memcpy(&_jointState, pMsg, sizeof(spi_data_t));
    }
    else
    {
        std::cout << "[ERROR] bzl_quadruped_sdk: "
                  << "HandleJointState!" << std::endl;
    }
    pthread_mutex_unlock(&_jointStateMutex);
}

void CommClient::HandleSensorImu(
    const lcm::ReceiveBuffer* pBuf,
    const std::string& str,
    const microstrain_lcmt* pMsg) {

    (void)pBuf;
    (void)str;
    pthread_mutex_lock(&_sensorStateMutex);
    memcpy(&_sensorState.imu.acc, pMsg->acc, sizeof(_sensorState.imu.acc));
    memcpy(&_sensorState.imu.gyro, pMsg->omega, sizeof(_sensorState.imu.gyro));
    memcpy(&_sensorState.imu.quat, pMsg->quat, sizeof(_sensorState.imu.quat));
    memcpy(&_sensorState.imu.rpy, pMsg->rpy, sizeof(_sensorState.imu.rpy));
    pthread_mutex_unlock(&_sensorStateMutex);
}