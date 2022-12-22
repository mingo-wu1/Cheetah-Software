/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData, 
	FSM_StateName stateNameIn, const std::string &stateStringIn)
    : FSM_State<T>(_controlFSMData, stateNameIn, stateStringIn)
{
  if(_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        //30 / (1000. * _controlFSMData->controlParameters->controller_dt),
        //22 / (1000. * _controlFSMData->controlParameters->controller_dt),
        27 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else if(_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        33 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else{
    assert(false);
  }


  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
  /// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
  _controlFSMData_ptr = _controlFSMData;
  /// Add End
}

template <typename T>
BT::NodeStatus FSM_State_Locomotion<T>::onStart() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cMPCOld->Initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  QUADRUPED_INFO(_logger, "On Enter");
  /// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
  if (_controlFSMData_ptr->_desiredStateCommand->gamepadCommand->back)
  {
	  _controlFSMData_ptr->userParameters->wbc_param_mode = 2;
  }
  else
  {
	  _controlFSMData_ptr->userParameters->wbc_param_mode = 1;
  }
  QUADRUPED_DEBUG(_logger, "wbc_param_mode: %d", (int)_controlFSMData_ptr->userParameters->wbc_param_mode);
  /// Add End
  return BT::NodeStatus::RUNNING;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
BT::NodeStatus FSM_State_Locomotion<T>::onRunning() {
  /// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
  SafeChecker();
  /// Add End

  // Call the locomotion control logic for this iteration
  LocomotionControlStep();

  /// Add Begin by lihao, wuchunming, 20211222, lock switch node for special transform
  bool transDone = true;
  for(int leg = 0; leg < 4; leg++)
    if(fabs(cMPCOld->contact_state[leg] - 0) < 1e-6)
      transDone = false;
  if(transDone == true){
    BZL::TransLock::Instance().done = true;
    return BT::NodeStatus::RUNNING;
  }
  
  BZL::TransLock::Instance().done = false;
  /// Add End
  return BT::NodeStatus::RUNNING;
}

extern rc_control_settings rc_control;

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  if(locomotionSafe()) {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_LOCOMOTION:
        break;
      /// Mod Begin by wuchunming, 2021-04-29, close locomotion state to balance stand
      /*
      case K_BALANCE_STAND:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::BALANCE_STAND;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;
      */
      /// Mod End
      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        this->transitionDuration = 0.;
        break;

      case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        break;

      case K_VISION:
        this->nextStateName = FSM_StateName::VISION;
        this->transitionDuration = 0.;
        break;

      default:
        if (this->transitionErrorMode ==
          (uint8_t)this->_data->controlParameters->control_mode)
        {
          break;
        }
        this->transitionErrorMode = (uint8_t)this->_data->controlParameters->control_mode;
        QUADRUPED_WARN(_logger, "Bad Request: Cannot transition from %d to %d",
          K_LOCOMOTION, (int)this->_data->controlParameters->control_mode);
    }
  } else {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
  }


  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    /// Mod Begin by wuchunming, 2021-04-29, close locomotion state to balance stand
    /*
    case FSM_StateName::BALANCE_STAND:
      LocomotionControlStep();

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }

      break;
    */
    /// Mod End
    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
      /// Mod Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      this->transitionData.done = cMPCOld->Transition();
      if(!this->transitionData.done) LocomotionControlStep();
      /// Ori Code:
      // this->transitionData.done = true;
      /// Mod End
      break;

    case FSM_StateName::RECOVERY_STAND:
      /// Mod Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      this->transitionData.done = cMPCOld->Transition();
      if(!this->transitionData.done) LocomotionControlStep();
      /// Ori Code:
      // this->transitionData.done = true;
      /// Mod End
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      QUADRUPED_WARN(_logger, "Something went wrong in transition");
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    QUADRUPED_WARN(_logger, "Unsafe locomotion: roll is %.3f degrees (max %.3f)", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    QUADRUPED_WARN(_logger, "Unsafe locomotion: pitch is %.3f degrees (max %.3f)", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      QUADRUPED_WARN(_logger, "Unsafe locomotion: leg %d is above hip (%.3f m)", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.18)) {
      QUADRUPED_WARN(_logger, "Unsafe locomotion: leg %d's y-position is bad (%.3f m)", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 9.) {
      QUADRUPED_WARN(_logger, "Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)", leg, v_leg);
      return false;
    }
  }

  return true;

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onHalted() {
  // Nothing to clean up when exiting
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();

  cMPCOld->Run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for(int leg(0); leg<4; ++leg){
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

  if(this->_data->userParameters->use_wbc > 0.9){
    _wbc_data->pBody_des = cMPCOld->pos_body_des;
    _wbc_data->vBody_des = cMPCOld->vel_body_des;
    _wbc_data->aBody_des = cMPCOld->acc_body_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pos_body_rpy_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vel_body_rpy_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = cMPCOld->pos_foot_world_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vel_foot_world_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->acc_foot_world_des[i];
// TODO(hanyuanqiang): Limited knee torque, to be deleted in the future
#if 1
#if (USE_RS485_A1 == 1)
    if (cMPCOld->contact_state[i]>0)
      cMPCOld->rforce_des[i][2] = 25.0;
    else
      cMPCOld->rforce_des[i] <<0.0, 0.0, 0.0;
#endif
#endif
      _wbc_data->Fr_des[i] = cMPCOld->rforce_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }
  for(int leg(0); leg<4; ++leg){
    //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }

}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg) {
  // Impedance control for the stance leg
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}

/// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
template <typename T>
void FSM_State_Locomotion<T>::SafeChecker(){
  auto gpCmd = this->_data->_desiredStateCommand->gamepadCommand;
  static int timer = 0;
  if(gpCmd->leftBumper ||
     gpCmd->rightBumper ||
     gpCmd->leftTriggerButton ||
     gpCmd->rightTriggerButton ||
     gpCmd->back ||
     gpCmd->start ||
     gpCmd->a ||
     gpCmd->b ||
     gpCmd->x ||
     gpCmd->y ||
     gpCmd->leftStickButton ||
     gpCmd->rightStickButton ||
     gpCmd->logitechButton ||
     gpCmd->buttonUp ||
     gpCmd->buttonDown ||
     gpCmd->buttonLeft ||
     gpCmd->buttonRight ||
     std::abs(gpCmd->leftStickAnalog[0]) > 1e-6 ||
     std::abs(gpCmd->leftStickAnalog[1]) > 1e-6 ||
     std::abs(gpCmd->rightStickAnalog[0]) > 1e-6 ||
     std::abs(gpCmd->rightStickAnalog[1]) > 1e-6 ||
     std::abs(gpCmd->leftTriggerAnalog) > 1e-6 ||
     std::abs(gpCmd->rightTriggerAnalog) > 1e-6
    )
  {
    timer = 0;
  }
  else
  {
    if(++timer >= 5 / this->_data->controlParameters->controller_dt){// 5s
      this->_data->_desiredStateCommand->rcCommand->mode = RC_mode::RECOVERY_STAND;
      timer = 0;
    }else{}
    auto data = this->_data->_stateEstimator->getResult();
    if(std::abs(data.vBody[0]) >= 0.2 || std::abs(data.vBody[1]) >= 0.2 || std::abs(data.rpy[0]) >= 0.05){
      timer = 0;
    }else{}
  }
}
/// Add End

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
