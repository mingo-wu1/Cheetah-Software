#include "BZL_Controller.hpp"

BZL_Controller::BZL_Controller():RobotController()
{
  BZL::SwitchConfig switchConfig;
  switchConfig_ = switchConfig.GetConfig();
  gpSwitchNode_ = std::make_shared<BZL::GamepadSwitchNode>();
}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void BZL_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters,
                                      _visualizationData, &userParameters,
                                      /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
                                      sensorData_, switchConfig_);


  /// Add by hanyuanqiang, 2021-06-18, Using SDK server interface
  _pCommServer = new BZL_QUADRUPED_SDK::CommServer(_controlFSM->data);
  /// Add end

  /// Add by wangjie237, hanyuanqiang, 2021-08-05, Add force detect
  _forceDetect = new ForceDetect<float>(_legController,
                                        _model,
                                        _stateEstimate,
                                        _controlParameters->controller_dt);
  /// Add end
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void BZL_Controller::runController() {
  // Find the current gait schedule
  _gaitScheduler->step();

  // Find the desired state trajectory
  /// Mod Begin by peibo, 2021-09-06, add handle tasks are not processed when adding automatic tasks
  if(_controlFSM->useGamePad() == true)
  {
    _desiredStateCommand->convertToStateCommands();
  }
  /// Ori Code:
  //_desiredStateCommand->convertToStateCommands();
  /// Mod End
  

  /// Add by hanyuanqiang, 2021-06-18, Using SDK server interface
  // Run SDK server
  _pCommServer->Run();
  /// Add end

  /// Add by wangjie237, hanyuanqiang, 2021-08-05, Add force detect
  _forceDetect->run();
  /// Add end

  // Run the Control FSM code
  //_controlFSM->runFSM();
  SwitchFSMState(_desiredStateCommand->rcCommand->mode, _controlParameters);
  switchConfig_.blackboard->set(BZL::FSM_STATE, gpSwitchNode_->SwitchFSMState(_desiredStateCommand->rcCommand->mode));

  //BZL::StatusChangeLogger logger_cout(_controlFSM);
  _controlFSM->executeTick();
}

void BZL_Controller::SwitchFSMState(int fsmState, RobotControlParameters* controlParameters){
  if(fsmState == RC_mode::STAND_UP){ //1
    controlParameters->control_mode = K_STAND_UP;
  }else if(fsmState == RC_mode::QP_STAND){ //2
    controlParameters->control_mode = K_BALANCE_STAND;
  }else if(fsmState == RC_mode::RECOVERY_STAND){ //3
    controlParameters->control_mode = K_RECOVERY_STAND;
  }else if(fsmState == RC_mode::PRONE){ //4
    controlParameters->control_mode = K_PRONE;
  }else if(fsmState == RC_mode::LOCOMOTION){ //5
    controlParameters->control_mode = K_LOCOMOTION;
  }else if(fsmState == RC_mode::BACKFLIP || fsmState == RC_mode::BACKFLIP_PRE){ //6
    controlParameters->control_mode = K_BACKFLIP;
  }else if(fsmState == RC_mode::FRONTJUMP){ //7
    controlParameters->control_mode = K_FRONTJUMP;
  }else if(fsmState == RC_mode::VISION){ //8
    controlParameters->control_mode = K_VISION;
  }else if(fsmState == RC_mode::JOINT_PD){ //9
    controlParameters->control_mode = K_JOINT_PD;
  }else if(fsmState == RC_mode::IMPEDANCE_CONTROL){ //10
    controlParameters->control_mode = K_IMPEDANCE_CONTROL;
  }else if(fsmState == RC_mode::PASSIVE){ //100
    controlParameters->control_mode = K_PASSIVE;
  }
}
