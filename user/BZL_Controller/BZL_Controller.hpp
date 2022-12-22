#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

#include <RobotController.h>
#include "Controllers/GaitScheduler.h"
#include "Controllers/ContactEstimator.h"
#include "FSM_States/ControlFSM.h"
#include "BZL_UserParameters.h"
#include "comm_server.h"
#include "Forcedetect/ForceDetect.h"
//#include <gui_main_control_settings_t.hpp>

class BZL_Controller: public RobotController{
public:
  BZL_Controller();
  virtual ~BZL_Controller() {
    /// Add by hanyuanqiang, 2021-06-18, Free unused memory
    delete _controlFSM;
    delete _gaitScheduler;
    delete _pCommServer;
    /// Add end
  }

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
  virtual void Estop(){ _controlFSM->initialize(); }


protected:
  ControlFSM<float>* _controlFSM = nullptr;
  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler = nullptr;
  /// Add by hanyuanqiang, 2021-06-18, Using SDK server interface
  BZL_QUADRUPED_SDK::CommServer* _pCommServer = nullptr;
  /// Add end
  ForceDetect<float>* _forceDetect = nullptr;
  BZL_UserParameters userParameters;

};


#endif
