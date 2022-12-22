/*!
 * @file    bzl_quadruped.cpp
 * @author  hanyuanqiang
 * @brief   BZL quadruped API entry, Contains the definition of command and state
 *
 */

#include <memory>

#include "bzl_quadruped_sdk/bzl_quadruped.h"
#include "quadruped_comm.h"
#include "comm_client.h"


using namespace BZL_QUADRUPED_SDK;

std::shared_ptr<QuadrupedComm> pQuadrupedComm;


BzlQuadruped::BzlQuadruped(){
    pQuadrupedComm = std::make_shared<CommClient>();
    pQuadrupedComm->Init();
}


BzlQuadruped::~BzlQuadruped(){

}

bool BzlQuadruped::SendCmd(const BodyCmd& bodyCmd){
    pQuadrupedComm->Send(bodyCmd);
    return true;
}

bool BzlQuadruped::SendCmd(const GamepadCmd& gamepadCmd){
    pQuadrupedComm->Send(gamepadCmd);
    return true;
}

bool BzlQuadruped::SendCmd(const JointCmd& jointCmd){
    pQuadrupedComm->Send(jointCmd);
    return true;
}

bool BzlQuadruped::ShutDown(void) {
    GamepadCmd gamepadCmd;
    gamepadCmd.rightBumper = 1;
    
    return SendCmd(gamepadCmd);
}


bool BzlQuadruped::GetState(BodyState& bodyState){
    pQuadrupedComm->Get(bodyState);
    return true;
}


bool BzlQuadruped::GetState(JointState& jointState){
    pQuadrupedComm->Get(jointState);
    return false;
}


bool BzlQuadruped::GetState(SensorState& sensorState){
    pQuadrupedComm->Get(sensorState);
    return true;
}
