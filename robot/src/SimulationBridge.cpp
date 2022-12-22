/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();

  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);

  // init Quadruped Controller

  try {
    QUADRUPED_INFO(_logger, "Starting main loop...");
    bool firstRun = true;
    for (;;) {
      // wait for our turn to access the shared memory
      // on the first loop, this gives the simulator a chance to put stuff in
      // shared memory before we start
      _sharedMemory().waitForSimulator();

      if (firstRun) {
        firstRun = false;
        // check that the robot type is correct:
        if (_robot != _sharedMemory().simToRobot.robotType) {
          QUADRUPED_INFO(_logger, 
            "simulator and simulatorDriver don't agree on which robot we are "
            "simulating (robot %d, sim %d)",
            (int)_robot, (int)_sharedMemory().simToRobot.robotType);
          throw std::runtime_error("robot mismatch!");
        }
      }

      // the simulator tells us which mode to run in
      _simMode = _sharedMemory().simToRobot.mode;
      switch (_simMode) {
        case SimulatorMode::RUN_CONTROL_PARAMETERS:  // there is a new control
          // parameter request
          handleControlParameters();
          break;
        case SimulatorMode::RUN_CONTROLLER:  // the simulator is ready for the
          // next robot controller run
          _iterations++;
          runRobotControl();
          break;
        case SimulatorMode::DO_NOTHING:  // the simulator is just checking to see
          // if we are alive yet
          break;
        case SimulatorMode::EXIT:  // the simulator is done with us
          QUADRUPED_INFO(_logger, "Transitioned to exit mode");
          return;
          break;
        default:
          throw std::runtime_error("unknown simulator mode");
      }

      // tell the simulator we are done
      _sharedMemory().robotIsDone();
    }
  } catch (std::exception& e) {
    /// Add by hanyuanqiang, 2021-03-30, Prevent errors in compiling high version GCC
    strncpy(_sharedMemory().robotToSim.errorMessage, e.what(), sizeof(_sharedMemory().robotToSim.errorMessage) - 1);
    /// Add End
    _sharedMemory().robotToSim.errorMessage[sizeof(_sharedMemory().robotToSim.errorMessage) - 1] = '\0';
    throw e;
  }

}

/*!
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters() {
  ControlParameterRequest& request =
      _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response =
      _sharedMemory().robotToSim.controlParameterResponse;
  if (request.requestNumber <= response.requestNumber) {
    // nothing to do!
    QUADRUPED_WARN(_logger, 
        "[SimulationBridge] Warning: the simulator has run a ControlParameter "
        "iteration, but there is no new request!");
    return;
  }

  // sanity check
  assert((request.requestNumber - response.requestNumber) == 1);

  response.nParameters = _robotParams.collection._map
                             .size();  // todo don't do this every single time?

  switch (request.requestKind) {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      param.set(request.value, request.parameterKind);

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      QUADRUPED_INFO(_logger, "%s", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      std::string name(request.name);
      if(!_userParams) {
        QUADRUPED_WARN(_logger, "tried to set user parameter, but the robot does not have any!");
      } else {
        ControlParameter& param = _userParams->collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        param.set(request.value, request.parameterKind);
      }

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      QUADRUPED_INFO(_logger, "%s", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;  // acknowledge
      response.parameterKind =
          request.parameterKind;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind =
          request.requestKind;  // just for debugging print statements

      QUADRUPED_INFO(_logger, "%s", response.toString().c_str());
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    QUADRUPED_INFO(_logger, "First run of robot controller...");
    if (_robotParams.isFullyInitialized()) {
      QUADRUPED_INFO(_logger, "\tAll %ld control parameters are initialized",
             _robotParams.collection._map.size());
    } else {
      QUADRUPED_ERROR(_logger, 
          "\tbut not all control parameters were initialized. Missing:\n%s",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    if(userControlParameters) {
      if (userControlParameters->isFullyInitialized()) {
        QUADRUPED_INFO(_logger, "\tAll %ld user parameters are initialized",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        QUADRUPED_ERROR(_logger, 
            "\tbut not all control parameters were initialized. Missing:\n%s",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }


    _robotRunner->driverCommand =
        &_sharedMemory().simToRobot.gamepadCommand;
    _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;
    _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;
    /// Add Begin by hanyuanqiang, 2021-03-24
#if (USE_RS485_A1 == 1)
    _robotRunner->rs485A1Data = &_sharedMemory().simToRobot.rs485A1Data;
#endif
    /// Add End
    _robotRunner->robotType = _robot;
    _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;
    _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    _robotRunner->tiBoardCommand =
        _sharedMemory().robotToSim.tiBoardCommand;
    /// Add Begin by hanyuanqiang, 2021-03-24
#if (USE_RS485_A1 == 1)
    _robotRunner->rs485A1Command = &_sharedMemory().robotToSim.rs485A1Command;
#endif
    /// Add End
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData =
        &_sharedMemory().robotToSim.visualizationData;
    _robotRunner->cheetahMainVisualization =
        &_sharedMemory().robotToSim.mainCheetahVisualization;

    /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
    _robotRunner->sensorData_ = &pressureData_;
    /// Add End
 
    _robotRunner->init();
    _firstControllerRun = false;

    /// Del Begin by wuchunming, 20210716, del for add serialport pressure sensor
    //sbus_thread = new std::thread(&SimulationBridge::run_sbus, this);
    /// Del End

    /// Add Begin by wuchunming, 20210716, add serialport pressure sensor
    //InitPressureSensor();
    /// Add End
  }
  _robotRunner->run();
}

/*!
 * Run the RC receive thread
 */
void SimulationBridge::run_sbus() {
  QUADRUPED_INFO(_logger, "starting...");
  int port = init_sbus(true);  // Simulation
  while (true) {
    if (port > 0) {
      int x = receive_sbus(port);
      if (x) {
        sbus_packet_complete();
      }
    }
    usleep(5000);
  }
}

/// Add Begin by wuchunming, 20210716, add serialport pressure sensor
void SimulationBridge::InitPressureSensor(){
  QUADRUPED_DEBUG(_logger, "Version : %s", serialPort_.getVersion().c_str());
  QUADRUPED_DEBUG(_logger, "availableFriendlyPorts : ");
  portName_ = "/dev/ttyUSB0";
  QUADRUPED_DEBUG(_logger, "select port name: %s", portName_.c_str());

  // readSlot_.SetSensorData(pressureData_);
  // serialport init and open
  serialPort_.init(portName_,
                  115200, 
                  itas109::Parity::ParityNone,
                  itas109::DataBits::DataBits8,
                  itas109::StopBits::StopOne,
                  itas109::FlowControl::FlowNone, 512);
  serialPort_.open();
  if(serialPort_.isOpened())
	{
	  QUADRUPED_DEBUG(_logger, "open success");	
	}
	else
	{
	  QUADRUPED_INFO(_logger, "open failed");
  }

  // only connect when data come and read, then write msg immediately
  serialPort_.readReady.connect(&readSlot_, &itas109::IOSlot::OnWriteMsg);
}
/// Add End
