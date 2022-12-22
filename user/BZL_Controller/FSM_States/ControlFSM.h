#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

// Checks the robot state and commands for safety
#include "SafetyChecker.h"

// FSM States
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_ImpedanceControl.h"
#include "../FSM_States/FSM_State_JointPD.h"
#include "../FSM_States/FSM_State_Locomotion.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_StandUp.h"
#include "../FSM_States/FSM_State_RecoveryStand.h"
#include "../FSM_States/FSM_State_Vision.h"
#include "../FSM_States/FSM_State_BackFlip.h"
#include "../FSM_States/FSM_State_FrontJump.h"
#include "Logger/Logger.h"

/// Add Begin by peibo, 2021-03-01, add prone mode
#include "../FSM_States/FSM_State_Prone.h"
/// Add End

/// Add Begin by peibo, 2021-08-04, add FSM_Task_Manager
#include "FSM_Task_Manager.h"
/// Add End

/// Add Begin
#include "../FSM_States/behavior_controller.h"
/// Add End
 
/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode { 
  NORMAL, TRANSITIONING, ESTOP, EDAMP };

/**
 *
 */
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_JointPD<T>* jointPD;
  FSM_State_ImpedanceControl<T>* impedanceControl;
  FSM_State_StandUp<T>* standUp;
  FSM_State_BalanceStand<T>* balanceStand;
  FSM_State_Locomotion<T>* locomotion;
  FSM_State_RecoveryStand<T>* recoveryStand;
  FSM_State_Vision<T>* vision;
  FSM_State_BackFlip<T>* backflip;
  FSM_State_FrontJump<T>* frontJump;

/// Add Begin by peibo, 2021-03-01, add prone mode
  FSM_State_Prone<T>* prone;
/// Add End
};


/**
 *
 */
template <typename T>
struct FSM_ControllerList {
};


/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM : public BZL::SwitchNode{
 public:
  ControlFSM(Quadruped<T>* _quadruped,
             StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController, GaitScheduler<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand,
             RobotControlParameters* controlParameters,
             VisualizationData* visualizationData,
             BZL_UserParameters* userParameters,
             /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
             itas109::SensorData* sensorData_,/// Add End
             const BT::NodeConfiguration& config
             );

  ~ControlFSM();

  // Initializes the Control FSM instance
//  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
//  void runFSM();

  // This will be removed and put into the SafetyChecker class
//  FSM_OperatingMode safetyPreCheck();

  //
//  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
//  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
//  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData<T> data;

  // FSM state information
//  FSM_StatesList<T> statesList;  // holds all of the FSM States
//  FSM_State<T>* currentState;    // current FSM state
//  FSM_State<T>* nextState;       // next FSM state
//  FSM_StateName nextStateName = FSM_StateName::INVALID;   // next FSM state name

  // Checks all of the inputs and commands for safety
  SafetyChecker<T>* safetyChecker;

//  TransitionData<T> transitionData;

  /// Add Begin by peibo, 2021-06-06, add handle tasks are not processed when adding automatic tasks
  bool useGamePad() {return _FSM_Task_Manager.getUseGamepad();};
  /// Add End

 private:
  // Operating mode of the FSM
//  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 10000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  int iter = 0;

  //lcm::LCM state_estimator_lcm;
  //state_estimator_lcmt _state_estimator;

  /// Add Begin by peibo, 2021-08-04, add FSM_Task_Manager
  FSM_Task_Manager<T> _FSM_Task_Manager;
  bool isUseFSMTaskManager = false;
  /// Add End
  BZL_QUADRUPED::Logger _logger;
  FSM_State_Locomotion<T>       *locomotion_;
  FSM_State_StandUp<T>          *standup_;
  FSM_State_BalanceStand<T>     *balancestand_;
  FSM_State_RecoveryStand<T>    *recoverystand_;
  FSM_State_Prone<T>            *prone_;
  FSM_State_BackFlip<T>         *backflip_;
  FSM_State_FrontJump<T>        *frontjump_;
  FSM_State_Vision<T>           *vision_;
  FSM_State_JointPD<T>          *jointpd_;
  FSM_State_ImpedanceControl<T> *impedancecontrol_;
  FSM_State_Passive<T>          *passive_;
};

#endif  // CONTROLFSM_H
