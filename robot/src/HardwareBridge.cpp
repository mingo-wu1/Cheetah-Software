/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef linux 

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"

#include "HardwareBridge.h"
//#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#if (USE_RS485_A1 == 1)
#include "rt/rt_rs485_a1.h"
#endif
#include "rt/rt_vectornav.h"
#include "rt/rt_ethercat.h"
#include "Utilities/Utilities_print.h"

#define USE_MICROSTRAIN
/// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor
//#define USE_LPMSUABAL2
/// Add End

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) {
  QUADRUPED_ERROR(_logger, "FAILED TO INITIALIZE HARDWARE: %s", reason);

  if (printErrno) {
    QUADRUPED_ERROR(_logger, "Error: %s", strerror(errno));
  }

  BZL_QUADRUPED::LoggerShutDown();
  exit(-1);
}

/*!
 * All hardware initialization steps that are common between Cheetah 3 and Mini Cheetah
 */
void HardwareBridge::initCommon() {
  QUADRUPED_INFO(_logger, "Init stack");
  prefaultStack();
  QUADRUPED_INFO(_logger, "Init scheduler");
  setupScheduler();
  if (!_interfaceLCM.good()) {
    initError("_interfaceLCM failed to initialize\n", false);
  }

  QUADRUPED_INFO(_logger, "Subscribe LCM");
  _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);
  _interfaceLCM.subscribe("interface_request",
                          &HardwareBridge::handleControlParameter, this);

  /// Add Begin by wuchunming02@countrygarden.com.cn, 2021-01-30, add lcm for spi cmd test
  _interfaceLCM.subscribe("testspicmd",
                          &HardwareBridge::handleTestSpiCmdLCM, this);
  QUADRUPED_INFO(_logger, "Start testspicmd LCM handler");
  /// Add End

  QUADRUPED_INFO(_logger, "Start interface LCM handler");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

/*!
 * Run interface LCM
 */
void HardwareBridge::handleInterfaceLCM() {
  while (!_interfaceLcmQuit) _interfaceLCM.handle();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  QUADRUPED_INFO(_logger, "Prefault stack...");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 */
void HardwareBridge::setupScheduler() {
  QUADRUPED_INFO(_logger, "Setup RT Scheduler...");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    initError("sched_setscheduler failed.\n", true);
  }
}

/*!
 * LCM Handler for gamepad message
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  _gamepadCommand.set(msg);
}

/// Add Begin by wuchunming02@countrygarden.com.cn, 2021-01-30, add lcm for spi cmd test
void HardwareBridge::handleTestSpiCmdLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const test_spi_command_t* msg) {
  (void)rbuf;
  (void)chan;
  _testSpiCmd.is_test = msg->is_test;
  for (int i = 0; i < 4; ++i) {
    _testSpiCmd.q_des_abad[i] = msg->q_des_abad[i];
    _testSpiCmd.q_des_hip[i] = msg->q_des_hip[i];
    _testSpiCmd.q_des_knee[i] = msg->q_des_knee[i];
    _testSpiCmd.qd_des_abad[i] = msg->qd_des_abad[i];
    _testSpiCmd.qd_des_hip[i] = msg->qd_des_hip[i];
    _testSpiCmd.qd_des_knee[i] = msg->qd_des_knee[i];
    _testSpiCmd.kp_abad[i] = msg->kp_abad[i];
    _testSpiCmd.kp_hip[i] = msg->kp_hip[i];
    _testSpiCmd.kp_knee[i] = msg->kp_knee[i];
    _testSpiCmd.kd_abad[i] = msg->kd_abad[i];
    _testSpiCmd.kd_hip[i] = msg->kd_hip[i];
    _testSpiCmd.kd_knee[i] = msg->kd_knee[i];
    _testSpiCmd.tau_abad_ff[i] = msg->tau_abad_ff[i];
    _testSpiCmd.tau_hip_ff[i] = msg->tau_hip_ff[i];
    _testSpiCmd.tau_knee_ff[i] = msg->tau_knee_ff[i];
    _testSpiCmd.flags[i] = msg->flags[i];
  }
}
/// Add End

/*!
 * LCM Handler for control parameters
 */
void HardwareBridge::handleControlParameter(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const control_parameter_request_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
    // nothing to do!
    QUADRUPED_WARN(_logger, 
        "[HardwareBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!");
    // return;
  }

  // sanity check
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
  if (nRequests != 1) {
    QUADRUPED_ERROR(_logger, "Hardware bridge: we've missed %ld requests",
           nRequests - 1);
  }

  switch (msg->requestKind) {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      if(!_userControlParameters) {
        QUADRUPED_WARN(_logger, "Got user param %s, but not using user parameters!",
               (char*)msg->name);
      } else {
        std::string name((char*)msg->name);
        ControlParameter& param = _userControlParameters->collection.lookup(name);

        // type check
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(
                  (ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);

        // respond:
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber;  // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind;  // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        //for debugging print statements
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());  // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        QUADRUPED_DEBUG(_logger, "[User Control Parameter] set %s to %s", name.c_str(),
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind)
                   .c_str());
      }
    } break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name((char*)msg->name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(
                (ControlParameterValueKind)msg->parameterKind));
      }

      // do the actual set
      ControlParameterValue v;
      memcpy(&v, msg->value, sizeof(v));
      param.set(v, (ControlParameterValueKind)msg->parameterKind);

      // respond:
      _parameter_response_lcmt.requestNumber =
          msg->requestNumber;  // acknowledge that the set has happened
      _parameter_response_lcmt.parameterKind =
          msg->parameterKind;  // just for debugging print statements
      memcpy(_parameter_response_lcmt.value, msg->value, 64);
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
      //for debugging print statements
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // just for debugging print statements
      _parameter_response_lcmt.requestKind = msg->requestKind;

      QUADRUPED_DEBUG(_logger, "[Robot Control Parameter] set %s to %s", name.c_str(),
             controlParameterValueToString(
                 v, (ControlParameterValueKind)msg->parameterKind)
                 .c_str());

    } break;

    default: {
      throw std::runtime_error("parameter type unsupported");
    }
    break;
  }
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}


MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(0)), 
#if (USE_RS485_A1 == 1)
      _rs485A1Lcm(getLcmUrl(0)),
#endif
      _microstrainLcm(getLcmUrl(1)),
      _rtVoice(&taskManager) {
  _load_parameters_from_file = load_parameters_from_file;
}

MiniCheetahHardwareBridge::~MiniCheetahHardwareBridge(){
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  if(_load_parameters_from_file) {
    QUADRUPED_INFO(_logger, "Loading parameters from file...");

    try {
      _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
      QUADRUPED_ERROR(_logger, "Failed to initialize robot parameters from yaml file: %s", e.what());
      BZL_QUADRUPED::LoggerShutDown();
      exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
      QUADRUPED_ERROR(_logger, "Failed to initialize all robot parameters");
      BZL_QUADRUPED::LoggerShutDown();
      exit(1);
    }

    QUADRUPED_INFO(_logger, "Loaded robot parameters");

    if(_userControlParameters) {
      try {
/// Change by hanyuanqiang, 2021-08-10, Add unitree RS485 A1 motor parameters
#if (USE_RS485_A1 == 1)
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters-rs485-a1.yaml");
#elif (USE_LINKAGE == 1)
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters-linkage.yaml");
#elif (USE_LINKAGE_INDUSTRIAL == 1)
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters-linkage-industrial.yaml");
#else
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
#endif
      } catch(std::exception& e) {
        QUADRUPED_ERROR(_logger, "Failed to initialize user parameters from yaml file: %s", e.what());
        BZL_QUADRUPED::LoggerShutDown();
        exit(1);
      }

      if(!_userControlParameters->isFullyInitialized()) {
        QUADRUPED_ERROR(_logger, "Failed to initialize all user parameters");
        BZL_QUADRUPED::LoggerShutDown();
        exit(1);
      }

      QUADRUPED_INFO(_logger, "Loaded user parameters");
    } else {
      QUADRUPED_WARN(_logger, "Did not load user parameters because there aren't any");
    }
  } else {
    QUADRUPED_INFO(_logger, "Loading parameters over LCM...");
    while (!_robotParams.isFullyInitialized()) {
      QUADRUPED_INFO(_logger, "Waiting for robot parameters...");
      usleep(1000000);
    }

    if(_userControlParameters) {
      while (!_userControlParameters->isFullyInitialized()) {
        QUADRUPED_INFO(_logger, "Waiting for user parameters...");
        usleep(1000000);
      }
    }
  }

  QUADRUPED_INFO(_logger, "Got all parameters, starting up!");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  /// Add Begin by hanyuanqiang, 2021-03-24, Judge whether to configure A1 motor
#if (USE_RS485_A1 == 1)
  _robotRunner->rs485A1Data = &_rs485A1Data;
  _robotRunner->rs485A1Command = &_rs485A1Command;
#else
  _robotRunner->spiData = &_spiData;
  _robotRunner->spiCommand = &_spiCommand;
#endif
  /// Add End

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  _robotRunner->sensorData_ = &pressureData_;
  /// Add End

  _firstRun = false;

  // init control thread

  statusTask.start();

  /// Add Begin by hanyuanqiang, 2021-03-24, Judge whether to configure A1 motor
#if (USE_RS485_A1 == 1)
  PeriodicMemberFunction<MiniCheetahHardwareBridge> rs485A1Task(
      &taskManager, .008, "rs485_a1", &MiniCheetahHardwareBridge::runRs485A1, this);
  rs485A1Task.start();
#else
  PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
      &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
  spiTask.start();
#endif
  /// Add End

  // microstrain
  if(_microstrainInit)
    _microstrainThread = std::thread(&MiniCheetahHardwareBridge::runMicrostrain, this);

  /// Add Begin by victor, 2021-01-21, add ig1 imu sensor implementation code
  if(_lpIG1ImuIsInit)
	_lpIG1ImuThread = std::thread(&MiniCheetahHardwareBridge::runLpIG1Imu,this);
  /// Add End

  /// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor implementation code
#if (USE_LPMSUSBAL2 == 1)
  if(_lpUSBAL2ImuInit)
  {
    _lpUSBAL2ImuThread = std::thread(&MiniCheetahHardwareBridge::runLpUSBAL2Imu,this);
  }
#endif
  /// Add End

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();
  
  if(_rtI2C.IsOk()){
    PeriodicMemberFunction<MiniCheetahHardwareBridge> i2cTask(
        &taskManager, .002, "i2c", &MiniCheetahHardwareBridge::runI2C, this);
    i2cTask.start();
  }

  // rc controller
  // Change by hanyuanqinag, 2021-09-18, Comment redundant threads
  // _port = init_sbus(false);  // Not Simulation
  // PeriodicMemberFunction<HardwareBridge> sbusTask(
  //     &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
  // sbusTask.start();

  // temporary hack: microstrain logger
  PeriodicMemberFunction<MiniCheetahHardwareBridge> microstrainLogger(
      &taskManager, .001, "microstrain-logger", &MiniCheetahHardwareBridge::logMicrostrain, this);
  microstrainLogger.start();

  /// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor implementation code
#if (USE_LPMSUSBAL2 == 1)
  PeriodicMemberFunction<MiniCheetahHardwareBridge> lpUSBAL2ImuLogger(
      &taskManager, .001, "LpUSBAL2Imu-logger", &MiniCheetahHardwareBridge::logLpUSBAL2Imu, this);
  lpUSBAL2ImuLogger.start();
#endif
  /// Add End
  for (;;) {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

/*!
 * Receive RC with SBUS
 */
void HardwareBridge::run_sbus() {
  if (_port > 0) {
    int x = receive_sbus(_port);
    if (x) {
      sbus_packet_complete();
    }
  }
}

void MiniCheetahHardwareBridge::runMicrostrain() {

  /// Add Begin by victor, 2021-01-21, add microstrain imu sensor implementation code
  /// ofile_mic.open("/home/user/workspace/temp/log/mic.txt");
  /// Add End

  while(true) {
    _microstrainImu.run();

#ifdef USE_MICROSTRAIN
_vectorNavData.accelerometer = _microstrainImu.acc;
_vectorNavData.quat[0] = _microstrainImu.quat[1];
_vectorNavData.quat[1] = _microstrainImu.quat[2];
_vectorNavData.quat[2] = _microstrainImu.quat[3];
_vectorNavData.quat[3] = _microstrainImu.quat[0];
_vectorNavData.gyro = _microstrainImu.gyro;
#endif

  /// Add Begin by victor, 2021-01-21, add microstrain imu sensor implementation code
  /*
  ofile<<_microstrainImu.acc[0]<<"	"<<_microstrainImu.acc[1]<<"	"<<_microstrainImu.acc[2]<<"	"<<_microstrainImu.quat[1]<<"	"<<_microstrainImu.quat[2]<<"	"<<_microstrainImu.quat[3]<<"	"<<_microstrainImu.quat[0]<<"	"<<_microstrainImu.gyro[0]<<"	"<<_microstrainImu.gyro[1]<<"	"<<_microstrainImu.gyro[2]<<"	"<<std::endl;
  */
  /// Add End
 
  }

  /// Add Begin by victor, 2021-01-21, add microstrain imu sensor implementation code
  /// ofile_mic.close();
  /// Add End

}

/// Add Begin by victor, 2021-01-21, add ig1 imu sensor implementation code
void MiniCheetahHardwareBridge::runLpIG1Imu(){

  /// ofile_ig1.open("/home/user/workspace/temp/log/ig.txt");

  while(true){
	_lpIG1Imu.run();
        /*
	_vectorNavData.accelerometer  = _lpIG1Imu.acc;
	_vectorNavData.quat[0]        = _lpIG1Imu.quat[1];
	_vectorNavData.quat[1]        = _lpIG1Imu.quat[2];
	_vectorNavData.quat[2]        = _lpIG1Imu.quat[3];
	_vectorNavData.quat[3]        = _lpIG1Imu.quat[0];
	_vectorNavData.gyro           = _lpIG1Imu.gyro;
        */
  /*
  ofile_ig1<<_lpIG1Imu.acc[0]<<"	"<<_lpIG1Imu.acc[1]<<"	"<<_lpIG1Imu.acc[2]<<"	"<<_lpIG1Imu.quat[1]<<"	"<<_lpIG1Imu.quat[2]<<"	"<<_lpIG1Imu.quat[3]<<"	"<<_lpIG1Imu.quat[0]<<"	"<<_lpIG1Imu.gyro[0]<<"	"<<_lpIG1Imu.gyro[1]<<"	"<<_lpIG1Imu.gyro[2]<<"	"<<std::endl;
  */
  }
  /// ofile_ig1.close();
}
/// Add End

/// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor definition code
void MiniCheetahHardwareBridge::runLpUSBAL2Imu()
{
#if (USE_LPMSUSBAL2 == 1)
  while(true)
  {
	  _lpUSBAL2Imu.update();
    _lpUSBAL2ImuMutex.lock();
      for(u32 i = 0; i < 4; i++) {
    _lpUSBAL2ImuData.quat[i] = _lpUSBAL2Imu.quat[i];
    }

    //Vec3<float> rpy = ori::quatToRPY(_lpUSBAL2Imu.quat);
    for(u32 i = 0; i < 3; i++) {
      _lpUSBAL2ImuData.rpy[i] = _lpUSBAL2Imu.rpy[i];
      _lpUSBAL2ImuData.acc[i] = _lpUSBAL2Imu.acc[i];
      _lpUSBAL2ImuData.omega[i] = _lpUSBAL2Imu.gyro[i];
    }
    _lpUSBAL2ImuMutex.unlock();

    _vectorNavData.accelerometer  = _lpUSBAL2Imu.acc;
    _vectorNavData.quat[0]        = _lpUSBAL2Imu.quat[1];
    _vectorNavData.quat[1]        = _lpUSBAL2Imu.quat[2];
    _vectorNavData.quat[2]        = _lpUSBAL2Imu.quat[3];
    _vectorNavData.quat[3]        = _lpUSBAL2Imu.quat[0];
    _vectorNavData.gyro           = _lpUSBAL2Imu.gyro;

    usleep(2000);
  }
#endif
}
/// Add End

void MiniCheetahHardwareBridge::logMicrostrain() {
  _microstrainImu.updateLCM(&_microstrainData);
  _microstrainLcm.publish("microstrain", &_microstrainData);
}

/// Add Begin by peibo, 2021-06-22, add usbal2 imu sensor definition code
void MiniCheetahHardwareBridge::logLpUSBAL2Imu() {
#if (USE_LPMSUSBAL2 == 1)
  _lpUSBAL2ImuMutex.lock();
  _lpUSBAL2ImuLcm.publish("lpUSBAL2Imu", &_lpUSBAL2ImuData);
  _lpUSBAL2ImuMutex.unlock();
#endif
}
/// Add End


/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahHardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
#ifndef USE_MICROSTRAIN
  QUADRUPED_INFO(_logger, "Init vectornav");
  if (!init_vectornav(&_vectorNavData)) {
    QUADRUPED_ERROR(_logger, "Vectornav failed to initialize");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif
/// Add Begin by peibo, 2021-06-22, add usbal2 imu 
#if (USE_LPMSUSBAL2 == 1)
  QUADRUPED_INFO(_logger, "Init LPMSUSBAL2");
  _lpUSBAL2ImuInit = _lpUSBAL2Imu.initImu();
#endif
/// Add End

#if (USE_RS485_A1 == 1)
  init_rs485_a1();
#else
  init_spi();
#endif

  _microstrainInit = _microstrainImu.tryInit(0, 921600);

  /// Add Begin by victor, 2021-01-21, add ig1 imu sensor implementation code
  bool has_IG1 = false;
  if(has_IG1){
    _lpIG1ImuIsInit = _lpIG1Imu.tryInit(0,115200);
  }
  /// Add End

  /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
  _rtI2C.Init(5, 0x11);
  /// Add End

  /// Add Begin by hanyuanqiang, 2021-04-29, add exception detection
  _checkError();
  /// Add End

}

void Cheetah3HardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
  QUADRUPED_INFO(_logger, "Init vectornav");
  if (!init_vectornav(&_vectorNavData)) {
    QUADRUPED_ERROR(_logger, "Vectornav failed to initialize\n");
    QUADRUPED_ERROR(_logger,      "\n****************\n"
                                  "**  WARNING!  **\n"
                                  "****************\n"
                                  "  IMU DISABLED  \n"
                                  "****************\n");
    //initError("failed to initialize vectornav!\n", false);
  }
}

/*!
 * Run Mini Cheetah SPI
 */
void MiniCheetahHardwareBridge::runSpi() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();

  memcpy(cmd, &_spiCommand, sizeof(spi_command_t));

  /// Add Begin by lihao and peibo, 2021-09-15, add the function that enable the motors again
  static int _cur_iter = 0, _start_iter = 2000;
  if (_robotRunner->driverCommand->rightBumper)
  {
    if (_isStop == false && _cur_iter == 0)
    {
      _isStop = true;
    }
    else if (_isStop == true && _cur_iter > _start_iter)
    {
      _checkError();
      if (false == _isStop)
      {
        QUADRUPED_INFO(_logger, "now enable");
        for (int i = 0; i < 4; ++i)
        {
          cmd->flags[i] = 1;
        }
      }
    }
    else
    {
      _cur_iter ++;
    }
  }
  else
  {
    _cur_iter = 0;
  }

  if(_isStop == true)
  {
    for(int i = 0; i < 4; ++i){
      cmd->kp_abad[i]     = 0;
      cmd->kp_hip[i]      = 0;
      cmd->kp_knee[i]     = 0;
      cmd->kd_abad[i]     = 0;
      cmd->kd_hip[i]      = 0;
      cmd->kd_knee[i]     = 0;
      cmd->tau_abad_ff[i] = 0;
      cmd->tau_hip_ff[i]  = 0;
      cmd->tau_knee_ff[i] = 0;
      cmd->flags[i]       = 0;
    }
  }else{}
  /// Add End

  spi_driver_run();
  memcpy(&_spiData, data, sizeof(spi_data_t));

  _spiLcm.publish("spi_data", data);
  _spiLcm.publish("spi_command", cmd);
}

#if (USE_RS485_A1 == 1)
void MiniCheetahHardwareBridge::runRs485A1() {
  rs485_a1_command_t* cmd = get_rs485_a1_command();
  rs485_a1_data_t* data = get_rs485_a1_data();

  memcpy(cmd, &_rs485A1Command, sizeof(rs485_a1_command_t));

  if(_robotRunner->driverCommand->rightBumper || _isStop ){
    _isStop = true;
    for (int i = 0; i < 4; ++i) {
      cmd->kp_abad[i]     = 0;
      cmd->kp_hip[i]      = 0;
      cmd->kp_knee[i]     = 0;
      cmd->kd_abad[i]     = 0;
      cmd->kd_hip[i]      = 0;
      cmd->kd_knee[i]     = 0;
      cmd->tau_abad_ff[i] = 0;
      cmd->tau_hip_ff[i]  = 0;
      cmd->tau_knee_ff[i] = 0;
      cmd->flags[i]       = 0;
    } 
  }

  rs485_a1_driver_run();
  memcpy(&_rs485A1Data, data, sizeof(rs485_a1_data_t));

  _rs485A1Lcm.publish("rs485_a1_data", data);
  _rs485A1Lcm.publish("rs485_a1_command", cmd);
}
#endif

/*!
 * Mini Cheetah exception detection
 */
void MiniCheetahHardwareBridge::_checkError() {

  _isStop = false;
  _rtLed.RtLedDisplay(COLOUR_PURPLE);
  
  /// Mod Begin by peibo, 2021-06-22, add usbal2 imu sensor header
  if (false == _microstrainInit && false == _lpUSBAL2ImuInit)
  /// Ori Code:
  //if (false == _microstrainInit)
  /// Mod End
  {
    _isStop = true;
    QUADRUPED_ERROR(_logger, "IMU init failed");
    _rtVoice.RtVoicePlay("error_imu_init.wav", 10.0);
    return;
  }

#if (USE_RS485_A1 == 1)
  // TODO(hanyuanqiang): reserve
#else
  const float q_adad_des[4] = {-0.6f, 0.6f, -0.6f, 0.6f};
  const float q_hip_des[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
  const float q_knee_des[4] = {2.7f, 2.7f, 2.7f, 2.7f};
  const float offset_error_abad = 0.15f;
  const float offset_error_hip = 0.20f;
  const float offset_error_knee = 0.15f;

  spi_data_t* data = get_spi_data();

  for (int i = 0; i < 10; i++)
  {
    usleep(50000);
    spi_driver_run();
  }

  for (int i = 0; i < 4; i++)
  {
    if ((fabsf(data->q_abad[i] - q_adad_des[i]) >= offset_error_abad) ||
        (fabsf(data->q_hip[i] - q_hip_des[i]) >= offset_error_hip) ||
        (fabsf(data->q_knee[i] - q_knee_des[i]) >= offset_error_knee))
    {
      _isStop = true;
      QUADRUPED_ERROR(_logger, "Leg init failed, \
        num = %d, abad = %f, hip = %f, knee = %f\n", 
        i, data->q_abad[i], data->q_hip[i], data->q_knee[i]);
      _rtVoice.RtVoicePlay("error_motor_position.wav", 10.0);
      return;
    }
  }
#endif
  _rtLed.RtLedDisplay(COLOUR_GREEN);
  _rtVoice.RtVoicePlay("dog.wav", 0.0);
}

void Cheetah3HardwareBridge::runEcat() {
  rt_ethercat_set_command(_tiBoardCommand);
  rt_ethercat_run();
  rt_ethercat_get_data(_tiBoardData);

  publishEcatLCM();
}

void Cheetah3HardwareBridge::publishEcatLCM() {
  for(int leg = 0; leg < 4; leg++) {
    ecatCmdLcm.x_des[leg] = _tiBoardCommand[leg].position_des[0];
    ecatCmdLcm.y_des[leg] = _tiBoardCommand[leg].position_des[1];
    ecatCmdLcm.z_des[leg] = _tiBoardCommand[leg].position_des[2];
    ecatCmdLcm.dx_des[leg] = _tiBoardCommand[leg].velocity_des[0];
    ecatCmdLcm.dy_des[leg] = _tiBoardCommand[leg].velocity_des[1];
    ecatCmdLcm.dz_des[leg] = _tiBoardCommand[leg].velocity_des[2];
    ecatCmdLcm.kpx[leg] = _tiBoardCommand[leg].kp[0];
    ecatCmdLcm.kpy[leg] = _tiBoardCommand[leg].kp[1];
    ecatCmdLcm.kpz[leg] = _tiBoardCommand[leg].kp[2];
    ecatCmdLcm.kdx[leg] = _tiBoardCommand[leg].kd[0];
    ecatCmdLcm.kdy[leg] = _tiBoardCommand[leg].kd[1];
    ecatCmdLcm.kdz[leg] = _tiBoardCommand[leg].kd[2];
    ecatCmdLcm.enable[leg] = _tiBoardCommand[leg].enable;
    ecatCmdLcm.zero_joints[leg] = _tiBoardCommand[leg].zero;
    ecatCmdLcm.fx_ff[leg] = _tiBoardCommand[leg].force_ff[0];
    ecatCmdLcm.fy_ff[leg] = _tiBoardCommand[leg].force_ff[1];
    ecatCmdLcm.fz_ff[leg] = _tiBoardCommand[leg].force_ff[2];
    ecatCmdLcm.tau_abad_ff[leg] = _tiBoardCommand[leg].tau_ff[0];
    ecatCmdLcm.tau_hip_ff[leg] = _tiBoardCommand[leg].tau_ff[1];
    ecatCmdLcm.tau_knee_ff[leg] = _tiBoardCommand[leg].tau_ff[2];
    ecatCmdLcm.q_des_abad[leg] = _tiBoardCommand[leg].q_des[0];
    ecatCmdLcm.q_des_hip[leg] = _tiBoardCommand[leg].q_des[1];
    ecatCmdLcm.q_des_knee[leg] = _tiBoardCommand[leg].q_des[2];
    ecatCmdLcm.qd_des_abad[leg] = _tiBoardCommand[leg].qd_des[0];
    ecatCmdLcm.qd_des_hip[leg] = _tiBoardCommand[leg].qd_des[1];
    ecatCmdLcm.qd_des_knee[leg] = _tiBoardCommand[leg].qd_des[2];
    ecatCmdLcm.kp_joint_abad[leg] = _tiBoardCommand[leg].kp_joint[0];
    ecatCmdLcm.kp_joint_hip[leg] = _tiBoardCommand[leg].kp_joint[1];
    ecatCmdLcm.kp_joint_knee[leg] = _tiBoardCommand[leg].kp_joint[2];
    ecatCmdLcm.kd_joint_abad[leg] = _tiBoardCommand[leg].kd_joint[0];
    ecatCmdLcm.kd_joint_hip[leg] = _tiBoardCommand[leg].kd_joint[1];
    ecatCmdLcm.kd_joint_knee[leg] = _tiBoardCommand[leg].kd_joint[2];
    ecatCmdLcm.max_torque[leg] = _tiBoardCommand[leg].max_torque;
  }

  for(int leg = 0; leg < 4; leg++) {
    ecatDataLcm.x[leg] = _tiBoardData[leg].position[0];
    ecatDataLcm.y[leg] = _tiBoardData[leg].position[1];
    ecatDataLcm.z[leg] = _tiBoardData[leg].position[2];
    ecatDataLcm.dx[leg] = _tiBoardData[leg].velocity[0];
    ecatDataLcm.dy[leg] = _tiBoardData[leg].velocity[1];
    ecatDataLcm.dz[leg] = _tiBoardData[leg].velocity[2];
    ecatDataLcm.fx[leg] = _tiBoardData[leg].force[0];
    ecatDataLcm.fy[leg] = _tiBoardData[leg].force[1];
    ecatDataLcm.fz[leg] = _tiBoardData[leg].force[2];
    ecatDataLcm.q_abad[leg] = _tiBoardData[leg].q[0];
    ecatDataLcm.q_hip[leg] = _tiBoardData[leg].q[1];
    ecatDataLcm.q_knee[leg] = _tiBoardData[leg].q[2];
    ecatDataLcm.dq_abad[leg] = _tiBoardData[leg].dq[0];
    ecatDataLcm.dq_hip[leg] = _tiBoardData[leg].dq[1];
    ecatDataLcm.dq_knee[leg] = _tiBoardData[leg].dq[2];
    ecatDataLcm.tau_abad[leg] = _tiBoardData[leg].tau[0];
    ecatDataLcm.tau_hip[leg] = _tiBoardData[leg].tau[1];
    ecatDataLcm.tau_knee[leg] = _tiBoardData[leg].tau[2];
    ecatDataLcm.tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
    ecatDataLcm.tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
    ecatDataLcm.tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
    ecatDataLcm.loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
    ecatDataLcm.ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
    ecatDataLcm.microtime_ti[leg] = _tiBoardData[leg].microtime_ti;
  }

  _ecatLCM.publish("ecat_cmd", &ecatCmdLcm);
  _ecatLCM.publish("ecat_data", &ecatDataLcm);
}

/*!
 * Send LCM visualization data
 */
void HardwareBridge::publishVisualizationLCM() {
  cheetah_visualization_lcmt visualization_data;
  for (int i = 0; i < 3; i++) {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];
  }

  for (int i = 0; i < 4; i++) {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  }

  for (int i = 0; i < 12; i++) {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];
  }

  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
}

Cheetah3HardwareBridge::Cheetah3HardwareBridge(RobotController *rc) : HardwareBridge(rc),  _ecatLCM(getLcmUrl(0)) {

}

void Cheetah3HardwareBridge::run() {
  initCommon();
  initHardware();

  printf("[Hardware Bridge] Loading parameters over LCM...\n");
  while (!_robotParams.isFullyInitialized()) {
    printf("[Hardware Bridge] Waiting for robot parameters...\n");
    usleep(1000000);
  }

  if(_userControlParameters) {
    while (!_userControlParameters->isFullyInitialized()) {
      printf("[Hardware Bridge] Waiting for user parameters...\n");
      usleep(1000000);
    }
  }

  printf("[Hardware Bridge] Got all parameters, starting up!\n");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->tiBoardData = _tiBoardData;
  _robotRunner->tiBoardCommand = _tiBoardCommand;
  _robotRunner->robotType = RobotType::CHEETAH_3;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  _robotRunner->vectorNavData = &_vectorNavData;

  _robotRunner->init();
  _firstRun = false;

  // init control thread

  statusTask.start();

  rt_ethercat_init();
  // Ecat Task start
  PeriodicMemberFunction<Cheetah3HardwareBridge> ecatTask(
      &taskManager, .001, "ecat", &Cheetah3HardwareBridge::runEcat, this);
  ecatTask.start();

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<Cheetah3HardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

  // rc controller disabled for now
//  _port = init_sbus(false);  // Not Simulation
//  PeriodicMemberFunction<HardwareBridge> sbusTask(
//      &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
//  sbusTask.start();


  for (;;) {
    usleep(100000);
    taskManager.printStatus();
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
void MiniCheetahHardwareBridge::runI2C(){
    _rtI2C.Read(&data, 8);
    _rtI2C.ConvertAd5593R(data, regVal, 4);

    memcpy(&i2c_data_, regVal, sizeof(regVal));
    i2cLcm_.publish("i2c_data", &i2c_data_);
}
/// Add End

#endif
