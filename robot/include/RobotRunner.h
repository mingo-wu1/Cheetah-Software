/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/StateEstimatorContainer.h"
#include "SimUtilities/IMUTypes.h"
#include "rt/rt_rc_interface.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "JPosInitializer.h"

#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "Utilities/PeriodicTask.h"
#include "cheetah_visualization_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "RobotController.h"
#include "Logger/Logger.h"
#include <lcm-cpp.hpp>

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
#include "CSerialPort/serialport_pressure_sensor.h"
/// Add End

class RobotRunner : public PeriodicTask {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController* , PeriodicTaskManager*, float, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator(bool cheaterMode = false);
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;

  GamepadCommand* driverCommand;
  RobotType robotType;
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  SpiData* spiData;
  SpiCommand* spiCommand;
  TiBoardCommand* tiBoardCommand;
  TiBoardData* tiBoardData;
#if (USE_RS485_A1 == 1)
  Rs485A1Data* rs485A1Data;
  Rs485A1Command* rs485A1Command;
#endif
  RobotControlParameters* controlParameters;
  VisualizationData* visualizationData;
  CheetahVisualization* cheetahMainVisualization;

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  itas109::SensorData *sensorData_ = nullptr;
  /// Add End

 private:
  float _ini_yaw;

  int iter = 0;

  void setupStep();
  void finalizeStep();

  JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;
  rc_control_settings rc_control;
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;
  state_estimator_lcmt state_estimator_lcm;
  leg_control_data_lcmt leg_control_data_lcm;
  // Contact Estimator to calculate estimated forces and contacts

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
  BZL_QUADRUPED::Logger _logger;
};

#endif  // PROJECT_ROBOTRUNNER_H
