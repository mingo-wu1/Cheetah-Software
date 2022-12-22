#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include <BZL_UserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
#include "CSerialPort/serialport_pressure_sensor.h"
/// Add End

/// Add Begin by peibo,anli, 2021-08-06,add automatic task parameters
typedef struct
{
  int balanceStandDanceNum = 1;
  bool backflipAutoTaskFlag = false;
}FSM_State_Auto_Task_Param;
/// Add End

/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  GaitScheduler<T>* _gaitScheduler;
  DesiredStateCommand<T>* _desiredStateCommand;
  RobotControlParameters* controlParameters;
  BZL_UserParameters* userParameters;
  VisualizationData* visualizationData;

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  itas109::SensorData* sensorData_;
  /// Add End

  /// Add Begin by peibo,anli, 2021-08-06,add automatic task parameters
  FSM_State_Auto_Task_Param autoTaskParam;
  /// Add End
};

/// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
typedef enum 
{
  //0: balance stand 1:locomotion  2:stair 
  BALANCE_STAND,
  LOCOMOTION,
  STAIR
}WBCParamMode;
/// Add End

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H
