/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux 

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

/// Add Begin by victor, 2021-01-21, add ig1 imu sensor header
#include "lpmsig1_interface/LpIG1Imu.h"
#include <fstream>
#include <iostream>
#include "test_spi_command_t.hpp"
/// Add End

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include "rt/rt_led.h"
#include "rt/rt_voice.h"
#include "Logger/Logger.h"

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
#include "CSerialPort/serialport_pressure_sensor.h"
#include "i2c_data_lcmt.hpp"
#include "rt/rt_i2c.h"
/// Add End

/// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor
/// Change by hanyuanqiang, wuchunming, 2021-08-04, Add usbal2 imu compilation macro
#if (USE_LPMSUSBAL2 == 1)
#include "lpmsusbal2_interface/LpUSBAL2Imu.h"
#endif
/// Add End

/*!
 * Interface between robot and hardware
 */
class HardwareBridge {
 public:
  HardwareBridge(RobotController* robot_ctrl)
  // Change by hanyuanqiang, 2021-09-18, Spare thread
      : statusTask(&taskManager, 20.0f),
        _interfaceLCM(getLcmUrl(1)),
        _visualizationLCM(getLcmUrl(1)),
        _logger("HardwareBridge") {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
    memset(&_spiCommand, 0, sizeof(SpiCommand));
    memset(&_spiData, 0, sizeof(SpiData));
#if (USE_RS485_A1 == 1)
    memset(&_rs485A1Command, 0, sizeof(Rs485A1Command));
    memset(&_rs485A1Data, 0, sizeof(Rs485A1Data));
#endif
  }
  void prefaultStack();
  void setupScheduler();
  void initError(const char* reason, bool printErrno = false);
  void initCommon();
  ~HardwareBridge() { delete _robotRunner; }
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const gamepad_lcmt* msg);

  /// Add Begin by wuchunming02@countrygarden.com.cn, 2021-01-30, add lcm for spi cmd test
  void handleTestSpiCmdLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const test_spi_command_t* msg);
  /// Add End

  void handleInterfaceLCM();
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);

  void publishVisualizationLCM();
  void run_sbus();

 protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus statusTask;
  GamepadCommand _gamepadCommand;
  VisualizationData _visualizationData;
  CheetahVisualization _mainCheetahVisualization;
  lcm::LCM _interfaceLCM;
  lcm::LCM _visualizationLCM;

  /// Add Begin by wuchunming02@countrygarden.com.cn, 2021-01-30, add lcm for spi cmd test
  lcm::LCM _testSpiCmdLCM;
  TestSpiCmd _testSpiCmd;
  /// Add End

  control_parameter_respones_lcmt _parameter_response_lcmt;
  SpiData _spiData;
  SpiCommand _spiCommand;

  TiBoardCommand _tiBoardCommand[4];
  TiBoardData _tiBoardData[4];
#if (USE_RS485_A1 == 1)
  Rs485A1Data _rs485A1Data;
  Rs485A1Command _rs485A1Command;
#endif
  bool _firstRun = true;
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams;
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;
  volatile bool _interfaceLcmQuit = false;
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;

  int _port;
  BZL_QUADRUPED::Logger _logger;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 */
class MiniCheetahHardwareBridge : public HardwareBridge {
 public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);
  ~MiniCheetahHardwareBridge();
  void runSpi();
#if (USE_RS485_A1 == 1)
  void runRs485A1();
#endif
  void initHardware();
  void run();
  void runMicrostrain();

  /// Add Begin by victor, 2021-01-21, add ig1 imu sensor definition code
  void runLpIG1Imu();
  /// Add End

  /// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor definition code
  void runLpUSBAL2Imu();
  void logLpUSBAL2Imu();
  /// Add End

  void logMicrostrain();
  void abort(const std::string& reason);
  void abort(const char* reason);

 private:
  VectorNavData _vectorNavData;
  lcm::LCM _spiLcm;
#if (USE_RS485_A1 == 1)
  lcm::LCM _rs485A1Lcm;
#endif
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;

  /// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor header
/// Change by hanyuanqiang, wuchunming, 2021-08-04, Add usbal2 imu compilation macro
#if (USE_LPMSUSBAL2 == 1)
  LpUSBAL2Imu _lpUSBAL2Imu;
  microstrain_lcmt _lpUSBAL2ImuData;
  lcm::LCM _lpUSBAL2ImuLcm;
  std::mutex _lpUSBAL2ImuMutex;
  std::thread _lpUSBAL2ImuThread;
#endif
  bool _lpUSBAL2ImuInit = false;
  /// Add End

  /// Add Begin by victor, 2021-01-21, add ig1 imu sensor definition code
  std::thread _lpIG1ImuThread;
  LpIG1Imu _lpIG1Imu;
  bool _lpIG1ImuIsInit = false;
  std::ofstream ofile_ig1;
  std::ofstream ofile_mic;
  /// Add End
  /// Add Begin by hanyuanqiang, 2021-04-29, add exception detection
  bool _isStop = false;
  void _checkError(void);
  /// Add End

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  std::string portName_;
  itas109::CSerialPort serialPort_;
  itas109::PressureData pressureData_;
  itas109::IOSlot readSlot_{serialPort_, pressureData_};

  void runI2C();
  i2c_data_lcmt i2c_data_;
  lcm::LCM i2cLcm_;
  RtI2C _rtI2C;
  int16_t regVal[4]{0};
  uint8_t data[8]{0};
  /// Add End

  RtLed _rtLed;
  RtVoice _rtVoice;

};

class Cheetah3HardwareBridge : public HardwareBridge {
public:
  Cheetah3HardwareBridge(RobotController* rc);
  void runEcat();
  void initHardware();
  void run();
  void publishEcatLCM();
  // todo imu?

private:
  VectorNavData _vectorNavData;
  lcm::LCM _ecatLCM;
  ecat_command_t ecatCmdLcm;
  ecat_data_t ecatDataLcm;
  // nothing?
};
#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
