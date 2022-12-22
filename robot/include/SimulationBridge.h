/*! @file SimulationBridge.h
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <thread>

#include "ControlParameters/RobotParameters.h"
#include "RobotRunner.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "Logger/Logger.h"

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
#include "CSerialPort/serialport_pressure_sensor.h"
/// Add End

class SimulationBridge {
 public:
  explicit SimulationBridge(RobotType robot, RobotController* robot_ctrl) : 
    _robot(robot),
    _logger("SimulationBridge") {
    _fakeTaskManager = new PeriodicTaskManager;
    _robotRunner = new RobotRunner(robot_ctrl, _fakeTaskManager, 0, "robot-task");
    _userParams = robot_ctrl->getUserControlParameters();

 }
  void run();
  void handleControlParameters();
  void runRobotControl();
  ~SimulationBridge() {
    delete _fakeTaskManager;
    delete _robotRunner;
  }
  void run_sbus();

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  std::string portName_;
  itas109::CSerialPort serialPort_;
  itas109::PressureData pressureData_;
  itas109::IOSlot readSlot_{serialPort_, pressureData_};
  void InitPressureSensor();
  /// Add End

 private:
  PeriodicTaskManager taskManager;
  bool _firstControllerRun = true;
  PeriodicTaskManager* _fakeTaskManager = nullptr;
  RobotType _robot;
  RobotRunner* _robotRunner = nullptr;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  RobotControlParameters _robotParams;
  ControlParameters* _userParams = nullptr;
  u64 _iterations = 0;
  BZL_QUADRUPED::Logger _logger;

/// Del Begin by wuchunming, 20210716, del for add serialport pressure sensor
//  std::thread* sbus_thread;
/// Del End
};

#endif  // PROJECT_SIMULATIONDRIVER_H
