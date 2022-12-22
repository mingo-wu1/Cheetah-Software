/*
 * @Author: your name
 * @Date: 2021-10-26 20:45:22
 * @LastEditTime: 2021-12-22 15:34:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Cheetah_BT_test/user/BZL_Controller/FSM_States/SafetyChecker.h
 */
#ifndef SAFETY_CHECKER_H
#define SAFETY_CHECKER_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

#include "../FSM_States/behavior_controller.h"

/**
 * The SafetyChecker handles the checks requested by the ControlFSM.
 */
template <typename T>
class SafetyChecker : public BZL::ActionNodeBase{
 public:
  SafetyChecker(ControlFSMData<T>* dataIn, const std::string &stateStringIn) 
    : BZL::ActionNodeBase(stateStringIn), data(dataIn){}

  BT::NodeStatus tick() override final{
      return BT::NodeStatus::SUCCESS;
  }

  void halt() override final{}

  // Pre checks to make sure controls are safe to run
  bool checkSafeOrientation();  // robot's orientation is safe to control

  // Post checks to make sure controls can be sent to robot
  bool checkPDesFoot();          // desired foot position is not too far
  bool checkForceFeedForward();  // desired feedforward forces are not too large

  // Stores the data from the ControlFSM
  ControlFSMData<T>* data;

 private:
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("SafetyChecker");
};

#endif  // SAFETY_CHECKER_H
