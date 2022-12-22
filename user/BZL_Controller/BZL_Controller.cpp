#include "BZL_Controller.hpp"

BZL_Controller::BZL_Controller():RobotController(){  }

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
                                      sensorData_
                                      /// Add End
                                      );


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
  _controlFSM->runFSM();
}


