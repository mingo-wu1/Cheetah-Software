/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <Configuration.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND") {
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;


  // Initialize GRF to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();

  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(1000.);
  /// Add Begin by peibo, 2021-09-28, add reading of dance action dat file
  initDanceDataMap(THIS_COM "config/dance/dance.txt");
  /// Add End
  /// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
  if(this->_data->controlParameters->use_cubicspline_for_qpstand_test){
    PlannerOfCubicSpline();
  }
  /// Add End
  /// Add Begin by peibo 2021-04-29,add WBC operation separation in different modes
	_controlFSMData_ptr = _controlFSMData;
	/// Add End
}

template <typename T>
void FSM_State_BalanceStand<T>::onEnter() {

  /// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
  r_TPData.setZero();
  p_TPData.setZero();
  y_TPData.setZero();
  TPCount = 0;
  heightOffset = 0;
  /// Add End

  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Always set the gait to be standing in this state
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
  
  _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

  if(_ini_body_pos[2] < 0.2) {
    _ini_body_pos[2] = 0.3;
  }

  last_height_command = _ini_body_pos[2];

  _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
  /// Add Begin by anli, 2021-09-15, add automatic dance function
  for (int i = 0; i < 20; i++)
  {
    danceOrder(0,i)=-1;
  }
  /// Add End
  /// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
  _controlFSMData_ptr->userParameters->wbc_param_mode = 0;
  QUADRUPED_INFO(_logger, "wbc_param_mode: %f", _controlFSMData_ptr->userParameters->wbc_param_mode);
  /// Add End
  this->transitionErrorMode = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStand<T>::run() {
  Vec4<T> contactState;
  contactState<< 0.5, 0.5, 0.5, 0.5;
  this->_data->_stateEstimator->setContactPhase(contactState);
  /// Add Begin by peibo,anli, 2021-09-15, add automatic dance function
  if (this->_data->_desiredStateCommand->gamepadCommand->rightTriggerButton){
    if (this->_data->controlParameters->use_cubicspline_for_qpstand_test != 1)
    {
      this->_data->controlParameters->use_cubicspline_for_qpstand_test = 1;
      TPCount = 0;
      this->_data->autoTaskParam.balanceStandDanceNum = 1;
      dance_num_change = true;
    }
  }
  /// Add End
  /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
  _controlFSMData_ptr->userParameters->Kp_joint_fr_wbc = _controlFSMData_ptr->userParameters->Kp_joint_fr_balanceStand;
  _controlFSMData_ptr->userParameters->Kd_joint_fr_wbc = _controlFSMData_ptr->userParameters->Kd_joint_fr_balanceStand;

  _controlFSMData_ptr->userParameters->Kp_joint_fl_wbc = _controlFSMData_ptr->userParameters->Kp_joint_fl_balanceStand;
  _controlFSMData_ptr->userParameters->Kd_joint_fl_wbc = _controlFSMData_ptr->userParameters->Kd_joint_fl_balanceStand;

  _controlFSMData_ptr->userParameters->Kp_joint_br_wbc = _controlFSMData_ptr->userParameters->Kp_joint_br_balanceStand;
  _controlFSMData_ptr->userParameters->Kd_joint_br_wbc = _controlFSMData_ptr->userParameters->Kd_joint_br_balanceStand;
  
  _controlFSMData_ptr->userParameters->Kp_joint_bl_wbc = _controlFSMData_ptr->userParameters->Kp_joint_bl_balanceStand;
  _controlFSMData_ptr->userParameters->Kd_joint_bl_wbc = _controlFSMData_ptr->userParameters->Kd_joint_bl_balanceStand;
  /// Add End
  BalanceStandStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition() {
  // Get the next state
  _iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_BALANCE_STAND:
      // Normal operation for state based transitions

      // Need a working state estimator for this
      /*if (velocity > v_max) {
        // Notify the State of the upcoming next state
        this->nextStateName = FSM_StateName::LOCOMOTION;

        // Transition instantaneously to locomotion state on request
        this->transitionDuration = 0.0;

        // Set the next gait in the scheduler to
        this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;

      }*/

      // TEST: in place to show automatic non user requested transitions
      /*if (_iter >= 5458) {
        this->nextStateName = FSM_StateName::LOCOMOTION;
        this->_data->controlParameters->control_mode = K_LOCOMOTION;
        this->transitionDuration = 0.0;
        this->_data->_gaitScheduler->gaitData._nextGait =
            GaitType::AMBLE;  // TROT; // Or get whatever is in
                              // main_control_settings
        _iter = 0;
      }*/
      break;

    case K_LOCOMOTION:
      // Requested change to balance stand
      this->nextStateName = FSM_StateName::LOCOMOTION;

      // Transition instantaneously to locomotion state on request
      this->transitionDuration = 0.0;

      // Set the next gait in the scheduler to
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_DAMP:  // normal c
      this->nextStateName = FSM_StateName::DAMP;
      this->transitionDuration = 0.0;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_BACKFLIP:
      this->nextStateName = FSM_StateName::BACKFLIP;
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
        K_BALANCE_STAND, (int)this->_data->controlParameters->control_mode);
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
TransitionData<T> FSM_State_BalanceStand<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::LOCOMOTION:
      BalanceStandStep();

      _iter++;
      if (_iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;
      
    case FSM_StateName::DAMP:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BACKFLIP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    default:
      QUADRUPED_ERROR(_logger, "Something went wrong in transition");
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceStand<T>::onExit() {
  _iter = 0;
  /// Add Begin by peibo, anli, 2021-09-15,add automatic dance function
	this->_data->controlParameters->use_cubicspline_for_qpstand_test = 0;
	/// Add End
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() {

  /// Add Begin by peibo,anli, 2021-09-15,add automatic dance function
  int curDanceNum = this->_data->autoTaskParam.balanceStandDanceNum;
  if(curDanceNum != dance_num)
  {
    dance_num = curDanceNum;
    dance_num_change = true;
  }
  /// Add End
  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->pBody_des[2] = _ini_body_pos[2] - 0.04; //Reduce the height of body to eliminate the vibration of the motor in balance stand mode.
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;
  if(this->_data->controlParameters->use_rc){
    const rc_control_settings* rc_cmd = this->_data->_desiredStateCommand->rcCommand;
    // Orientation
    _wbc_data->pBody_RPY_des[0] = rc_cmd->rpy_des[0]*1.4;
    _wbc_data->pBody_RPY_des[1] = rc_cmd->rpy_des[1]*0.46;
    _wbc_data->pBody_RPY_des[2] -= rc_cmd->rpy_des[2];

    // Height
    _wbc_data->pBody_des[2] += 0.12 * rc_cmd->height_variation;
  }else{

    /// Add Begin by peibo,anli, 2021-09-15,add automatic dance function
    if (dance_num_change)
    {
      PlannerOfCubicSpline();
      dance_num_change = false;
    }
    /// Add End
    /// Mod Begin by peibo, 2021-09-28, add reading of dance action dat file
    if(this->_data->controlParameters->use_cubicspline_for_qpstand_test && isDanceNumberEmpty(dance_num))
    {
     QUADRUPED_ERROR(_logger,"[FSM BalanceStand] Dance numer %d does not exist!", dance_num);
      this->_data->controlParameters->use_cubicspline_for_qpstand_test = 0;
    }
    /// Add End
    /// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
    if(this->_data->controlParameters->use_cubicspline_for_qpstand_test){
      InterpolatorOfCubicSpline();
    }else{
    /// Add End

    /// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
    // Orientation
      _wbc_data->pBody_RPY_des[0] = 
       0.4 * TrajectoryPlan(r_TPData, this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0], 1);

      _wbc_data->pBody_RPY_des[1] = 
       0.4 * TrajectoryPlan(p_TPData, this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0], 1);
    
      _wbc_data->pBody_RPY_des[2] -= 
       0.6 * TrajectoryPlan(y_TPData, this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1], 1);
    
    // Height
      /// Add Begin by wuchunming, 2021-06-02, add body height offset
      heightOffset += 0.0005 * (bool)this->_data->_desiredStateCommand->gamepadCommand->buttonRight;
      heightOffset -= 0.0005 * (bool)this->_data->_desiredStateCommand->gamepadCommand->buttonLeft;
      heightOffset = heightOffset >= -0.1 ? (heightOffset <= 0 ? heightOffset : 0) : -0.1;
      _wbc_data->pBody_des[2] += 0.2 * _wbc_data->pBody_RPY_des[1] + heightOffset;
      /// origin code
      // _wbc_data->pBody_des[2] += 0.2 * _wbc_data->pBody_RPY_des[1];
      /// Add End
      
    }

    /* origin code
    // Orientation
    _wbc_data->pBody_RPY_des[0] = 
     0.6* this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
     _wbc_data->pBody_RPY_des[1] = 
      0.6*this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
    
    /// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
    _wbc_data->pBody_RPY_des[2] -= 
     this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1];

    // Height
    _wbc_data->pBody_des[2] += 
      0.12 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
    */
    /// Add End
  }
  _wbc_data->vBody_Ori_des.setZero();

  for(size_t i(0); i<4; ++i){
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight/4.;
    _wbc_data->contact_state[i] = true;
  }
  
  if(this->_data->_desiredStateCommand->trigger_pressed) {
    _wbc_data->pBody_des[2] = 0.05;

    if(last_height_command - _wbc_data->pBody_des[2] > 0.001) {
      _wbc_data->pBody_des[2] = last_height_command - 0.001;
    }
  }
  last_height_command = _wbc_data->pBody_des[2];

  _wbc_ctrl->run(_wbc_data, *this->_data);
}

/// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
/**
 * @brief Cubic Interpolation Method for Trajectory Plan
 * @param [in] gamepad_des_theta, gamepad destination theta
 * @param [in] plan_deltaT, trajectory plan deltaT
 * @return plan_body_des_theta, trajectory plan destination theta of body 
 */
template <typename T>
T FSM_State_BalanceStand<T>::TrajectoryPlan(TrajectoryPlanData<T> &data, T _gamepad_des_theta, T _plan_deltaT){
  (void)_plan_deltaT;
  data.gamepad_des_theta = _gamepad_des_theta;

  if(data.gamepad_des_theta - data.gamepad_pre_theta > 0.000001 || data.gamepad_des_theta - data.gamepad_pre_theta < -0.000001 ){
      data.plan_time_step = 0.0;
      data.plan_deltaT = 0.5; /* std::fabs(data.gamepad_des_theta - data.plan_body_des_theta);*/
      data.plan_body_pre_theta = data.plan_body_des_theta;
  }else{}

  if(data.plan_time_step < data.plan_deltaT){
    data.plan_time_step += this->_data->controlParameters->controller_dt ? this->_data->controlParameters->controller_dt : 0.002;
    data.plan_body_des_theta = data.plan_body_pre_theta + 0 * data.plan_time_step + 
            3 * (data.gamepad_des_theta - data.plan_body_pre_theta ) / ( data.plan_deltaT * data.plan_deltaT ) * data.plan_time_step * data.plan_time_step + 
            (-2) * (data.gamepad_des_theta - data.plan_body_pre_theta) / ( data.plan_deltaT * data.plan_deltaT * data.plan_deltaT ) * data.plan_time_step * data.plan_time_step * data.plan_time_step;
  }else{}

  data.gamepad_pre_theta = data.gamepad_des_theta;

  return data.plan_body_des_theta;
}
/// Add End

/// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
/**
 * @brief Encapsulation of Cubic Spline Planner
 */
template <typename T>
void FSM_State_BalanceStand<T>::PlannerOfCubicSpline(){
  /// Mod Begin by anli, 2021-09-15, add automatic dance function
  float ts;
  ts = this->_data->controlParameters->controller_dt ? this->_data->controlParameters->controller_dt : 0.002;
  TPCount = 0;
  /// Mod Begin by peibo, 2021-09-28, add reading of dance action dat file
  auto iter = allDanceDataMap.find(dance_num);
  if(iter != allDanceDataMap.end())
  {
    splineGenData = cs.SplineGenerator(iter->second.q * 0.4, iter->second.t, ts);
  }
  /// Ori Code:
    // switch (dance_num)
    // {
    // case 1:
    //   dance.init(71);
    //   dance.q << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
    //       0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.029, -0.048, -0.048, -0.029, -0.000, 0.029, 0.048, 0.048, 0.029, 0.000, -0.029, -0.048, -0.048, -0.029, -0.000, 0.029, 0.048, 0.048, 0.029, 0.000, -0.029, -0.048, -0.048, -0.029, -0.000, 0.029, 0.048, 0.048, 0.029, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.035, 0.057, 0.057, 0.035, 0.000, -0.035, -0.057, -0.057, -0.035, -0.000,
    //       0.000, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.011, -0.041, -0.079, -0.109, -0.120, -0.109, -0.079, -0.041, -0.011, 0.000,
    //       0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.294, -0.476, -0.476, -0.294, -0.000, 0.294, 0.476, 0.476, 0.294, 0.000, -0.294, -0.476, -0.476, -0.294, -0.000, 0.294, 0.476, 0.476, 0.294, 0.000, -0.294, -0.476, -0.476, -0.294, -0.000, 0.294, 0.476, 0.476, 0.294, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
    //       0.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -0.100, -0.200, -0.300, -0.400, -0.500, -0.600, -0.700, -0.800, -0.900, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -1.000, -0.900, -0.800, -0.700, -0.600, -0.500, -0.400, -0.300, -0.200, -0.100, 0.000, 0.000, -0.485, -0.185, 0.185, 0.485, 0.600, 0.485, 0.185, -0.185, -0.485, -0.600,
    //       0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.235, 0.380, 0.380, 0.235, 0.000, -0.235, -0.380, -0.380, -0.235, -0.000, 0.235, 0.380, 0.380, 0.235, 0.000, -0.235, -0.380, -0.380, -0.235, -0.000, 0.235, 0.380, 0.380, 0.235, 0.000, -0.235, -0.380, -0.380, -0.235, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.353, 0.571, 0.571, 0.353, 0.000, -0.353, -0.571, -0.571, -0.353, -0.000;
    //   dance.t << 0.00, 1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 9.05, 9.10, 9.15, 9.20, 9.25, 9.30, 9.35, 9.40, 9.45, 9.50, 9.68, 9.86, 10.04, 10.22, 10.40, 10.58, 10.76, 10.94, 11.12, 11.30, 11.48, 11.66, 11.84, 12.02, 12.20, 12.38, 12.56, 12.74, 12.92, 13.10, 13.28, 13.46, 13.64, 13.82, 14.00, 14.18, 14.36, 14.54, 14.72, 14.90, 14.95, 15.00, 15.05, 15.10, 15.15, 15.20, 15.25, 15.30, 15.35, 15.40, 16.40, 16.80, 17.20, 17.60, 18.00, 18.40, 18.80, 19.20, 19.60, 20.00, 20.40;
    //   dance.q = 0.4 * dance.q;
    //   break;
    // case 2:
    //   dance.init(73);
    //   dance.q << 0.000, 0.000, 0.000, 0.032, 0.000, 0.000, 0.032, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.050, 0.000, 0.050, 0.000, 0.000, 0.000, 0.000, 0.050, 0.000, 0.050, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
    //       0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.050, 0.000, 0.050, 0.000, 0.000, 0.000, 0.000, -0.050, 0.000, -0.050, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.050, 0.000, -0.050, 0.000, 0.000, 0.050, 0.000, -0.050, 0.000, 0.000,
    //       0.000, -0.070, -0.070, -0.049, -0.070, -0.070, -0.049, -0.070, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.096, 0.000, -0.096, 0.000, -0.096, 0.000, 0.000, -0.096, 0.000, -0.096, 0.000, -0.096, 0.000, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, 0.000, 0.000, 0.049, -0.070, 0.049, 0.000, 0.000, 0.049, -0.070, 0.049, 0.000, 0.000,
    //       0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.800, -0.800, 0.800, 0.800, -0.800, -0.800, 0.800, 0.800, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, 0.000, 0.000, 1.000, 0.000, -1.000, 0.000, 0.000, 1.000, 0.000, -1.000, 0.000, 0.000,
    //       0.000, -1.000, -1.000, -1.500, -1.000, -1.000, -1.500, -1.000, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, 0.000, -0.800, 0.800, -0.800, 0.000, 0.000, -0.800, 0.800, -0.800, 0.000, 0.000,
    //       0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000;
    //   dance.t << 0.00, 1.00, 1.50, 1.70, 2.70, 3.20, 3.40, 4.40, 5.40, 6.40, 6.90, 7.90, 8.40, 9.40, 9.90, 10.90, 11.40, 12.40, 13.40, 14.40, 15.40, 16.40, 17.40, 18.40, 19.40, 20.40, 21.40, 22.40, 23.40, 24.40, 25.40, 26.40, 27.40, 28.40, 29.40, 30.40, 31.40, 32.40, 33.40, 34.40, 35.40, 36.40, 37.40, 38.40, 39.40, 40.40, 41.40, 42.40, 43.40, 44.40, 45.40, 46.40, 47.40, 48.40, 49.40, 50.40, 51.40, 52.40, 53.40, 54.40, 55.40, 56.40, 57.40, 58.40, 59.40, 60.40, 61.40, 62.40, 63.40, 64.40, 65.40, 66.40, 67.40;
    //   dance.q = 0.4 * dance.q;
    //   break;
    // case 0:
    //   danceOrder.block(0, 0, 1, 15) << 0, 1, 0, 2, 0, 0, 3, 0, 4, 4, 0, 5, 5, 0, 0;
    //   danceCodeGen();
    //   break;

    // default:
    //   break;
    // }
    // splineGenData = cs.SplineGenerator(dance.q, dance.t, ts);
  /// Mod End 
  ///Ori Code:
  // Eigen::Matrix<float, 4, 61> q;
  // Eigen::Matrix<float, 1, 61> t;
  // float ts; 

  // // q << 0, 0,-0.7, 0, 0.7, 0, -0.7, 0, 0,//yaw
  // //      0, 0, 0, 1.2, 0, -1.5, 0, 0, 0,//pitch
  // //      0, -0.07, -0.07, -0.07, -0.07, -0.07, -0.07, -0.07, 0,//height
  // //      0, 0, 0, 0, 0, 0, 0, 0, 0;
  // q << 0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,-0.700,0.700,-0.700,0.700,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.700,0.000,-0.700,0.000,0.000,0.000,
  //      0.000,0.000,-1.000,-1.000,-1.500,-1.000,-1.000,-1.500,-1.000,0.000,0.000,1.000,1.000,1.000,1.000,0.000,0.000,-1.100,-1.100,-1.100,-1.100,-1.100,-1.100,-1.100,-1.100,0.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.200,-1.000,0.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,-1.000,0.000,0.000,
  //      0.000,0.000,-0.070,-0.070,-0.049,-0.070,-0.070,-0.049,-0.070,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,-0.060,-0.060,-0.060,-0.060,-0.060,-0.060,-0.060,-0.060,0.000,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.040,-0.080,0.000,-0.020,0.040,-0.020,0.040,-0.020,0.040,-0.020,0.040,-0.020,0.040,-0.020,0.040,-0.020,0.040,-0.020,0.040,0.000,0.000,
  //      0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,-1.000,-1.000,1.000,1.000,-1.000,-1.000,1.000,1.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,-1.000,0.000,1.000,0.000,-1.000,0.000,1.000,0.000,-1.000,0.000,1.000,0.000,-1.000,0.000,1.000,0.000,0.000,0.000;
  
  // // t << 0, 1, 2, 3, 4, 5, 6, 7, 8;
  // // t = 1.2 * t;
  // t << 0.000,1.000,2.000,2.500,2.700,3.700,4.200,4.400,5.400,6.400,7.400,8.400,9.400,10.400,11.400,12.400,12.500,13.500,14.000,15.000,15.500,16.500,17.000,18.000,18.500,19.500,20.500,21.500,22.500,23.500,24.500,25.500,26.500,27.500,28.500,29.500,30.500,31.500,32.500,33.500,34.500,35.500,36.500,37.500,38.500,39.500,40.500,41.500,42.500,43.500,44.500,45.500,46.500,47.500,48.500,49.500,50.500,51.500,52.500,53.500,54.500;
  // t = 1 * t;

  // ts = this->_data->controlParameters->controller_dt ? this->_data->controlParameters->controller_dt : 0.002;
  // TPCount = 0;
  // if(cs.CubicSplineDataCheck(q, t, ts)){
  //   splineGenData = cs.SplineGenerator(q, t, ts);
  // }else{
  //   std::cerr << "data of cubic spline is bad."<<std::endl;
  // }
  /// Mod End
}

/**
 * @brief Encapsulation of Cubic Spline Interpolator
 */
template <typename T>
void FSM_State_BalanceStand<T>::InterpolatorOfCubicSpline(){
  /// Mod Begin by anli, 2021-09-15, add automatic dance function
  if (TPCount < 0)
	{
		_wbc_data->pBody_des[0] = _ini_body_pos[0];
		_wbc_data->pBody_des[1] = _ini_body_pos[1];
		_wbc_data->pBody_des[2] = _ini_body_pos[2];
		_wbc_data->pBody_RPY_des[0] = _ini_body_ori_rpy[0];
		_wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1];
		_wbc_data->pBody_RPY_des[2] = _ini_body_ori_rpy[2];
		TPCount++;
	}
  else if (cs.GetDataCheck())
  {
    _wbc_data->pBody_des[0] = _ini_body_pos[0]+splineGenData.cs_q(0, TPCount);
    _wbc_data->pBody_des[1] = _ini_body_pos[1]+splineGenData.cs_q(1, TPCount);
    _wbc_data->pBody_des[2] = _ini_body_pos[2] +splineGenData.cs_q(2, TPCount);
    _wbc_data->pBody_RPY_des[0] =_ini_body_ori_rpy[0]+ 0.4 * splineGenData.cs_q(3, TPCount);
    _wbc_data->pBody_RPY_des[1] =_ini_body_ori_rpy[1]+ 0.4 * 0.6 * splineGenData.cs_q(4, TPCount);
    _wbc_data->pBody_RPY_des[2] =_ini_body_ori_rpy[2]+0.4 * splineGenData.cs_q(5, TPCount);
    TPCount++;

    if(TPCount >= splineGenData.cs_q.cols()){
      this->_data->controlParameters->use_cubicspline_for_qpstand_test = 0;
      TPCount=-100;
    }
  }
  else
  {
    std::cerr << "data of cubic spline is bad." << std::endl;
  }
  /// Ori Code:
  // if(cs.GetDataCheck()){
  //   _wbc_data->pBody_RPY_des[0] = 0.4 * splineGenData.cs_q(3, TPCount);
  //   _wbc_data->pBody_RPY_des[1] = 0.4 * 0.6 * splineGenData.cs_q(1, TPCount);
  //   _wbc_data->pBody_RPY_des[2] = 0.4 * splineGenData.cs_q(0, TPCount);
  //   _wbc_data->pBody_des[2] += splineGenData.cs_q(2, TPCount);
  //   TPCount ++;
  //   TPCount = TPCount >= splineGenData.cs_q.cols() ? 0 : TPCount;
  // }else{
  //   std::cerr << "data of cubic spline is bad."<<std::endl;
  // }
  /// Mod End
}
/// Add End
/// Add Begin by anli, 2021-09-15, add automatic dance function
/**
 * @brief generate transitions between different actions
 */
template <typename T>
void FSM_State_BalanceStand<T>::danceCodeGen()
{
  if (first_gen_dance_single)
  {
    DanceSingle[0].col = 1; //temp
    DanceSingle[0].q.block(0, 0, 6, DanceSingle[0].col) << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[0].t.block(0, 0, 1, DanceSingle[0].col) << 1.00;

    DanceSingle[1].col = 8; //sit_jump
    DanceSingle[1].q.block(0, 0, 6, DanceSingle[1].col) << 0.000, 0.000, 0.000, 0.032, 0.000, 0.000, 0.032, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, -0.070, -0.070, -0.049, -0.070, -0.070, -0.049, -0.070,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, -1.000, -1.000, -1.500, -1.000, -1.000, -1.500, -1.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[1].t.block(0, 0, 1, DanceSingle[1].col) << 1.00, 2.00, 2.50, 2.70, 3.70, 4.20, 4.40, 5.40;

    DanceSingle[2].col = 4; //smell
    DanceSingle[2].q.block(0, 0, 6, DanceSingle[2].col) << 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000,
        1.000, 1.000, 1.000, 1.000,
        -0.700, 0.700, -0.700, 0.700;
    DanceSingle[2].t.block(0, 0, 1, DanceSingle[2].col) << 1.00, 2.00, 3.00, 4.00;

    DanceSingle[3].col = 8; //swing head;
    DanceSingle[3].q.block(0, 0, 6, DanceSingle[3].col) << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050,
        -0.800, -0.800, 0.800, 0.800, -0.800, -0.800, 0.800, 0.800,
        -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[3].t.block(0, 0, 1, DanceSingle[3].col) << 1.00, 1.50, 2.50, 3.00, 4.00, 4.50, 5.50, 6.00;

    DanceSingle[4].col = 8; //dance
    DanceSingle[4].q.block(0, 0, 6, DanceSingle[4].col) << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000,
        0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000;
    DanceSingle[4].t.block(0, 0, 1, DanceSingle[4].col) << 1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00;

    DanceSingle[5].col = 8; //dance1
    DanceSingle[5].q.block(0, 0, 6, DanceSingle[5].col) << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080,
        -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000,
        0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000,
        0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000;
    DanceSingle[5].t.block(0, 0, 1, DanceSingle[5].col) << 1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00;

    DanceSingle[6].col = 5; //dance_left_right
    DanceSingle[6].q.block(0, 0, 6, DanceSingle[6].col) << 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.050, 0.000, -0.050, 0.000,
        0.000, 0.049, -0.070, 0.049, 0.000,
        0.000, 1.000, 0.000, -1.000, 0.000,
        0.000, -0.800, 0.800, -0.800, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[6].t.block(0, 0, 1, DanceSingle[6].col) << 1.00, 2.00, 3.00, 4.00, 5.00;

    DanceSingle[7].col = 6; //dance
    DanceSingle[7].q.block(0, 0, 6, DanceSingle[7].col) << 0.000, 0.050, 0.000, 0.050, 0.000, 0.000,
        0.000, 0.050, 0.000, 0.050, 0.000, 0.000,
        -0.096, 0.000, -0.096, 0.000, -0.096, 0.000,
        0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
        0.000, -0.000, 0.000, -0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[7].t.block(0, 0, 1, DanceSingle[7].col) << 1.00, 2.00, 3.00, 4.00, 5.00, 6.00;

    DanceSingle[8].col = 5; //dance
    DanceSingle[8].q.block(0, 0, 6, DanceSingle[8].col) << 0.000, 0.050, 0.000, 0.050, 0.000,
        0.000, -0.050, 0.000, -0.050, 0.000,
        -0.096, 0.000, -0.096, 0.000, -0.096,
        0.000, 0.000, 0.000, -0.000, 0.000,
        0.000, -0.000, 0.000, -0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000;
    DanceSingle[8].t.block(0, 0, 1, DanceSingle[8].col) << 1.00, 2.00, 3.00, 4.00, 5.00;

    first_gen_dance_single = false;
  }
  int total_num = 0;
  int total_col = 0;
  for (int i = 0; i < 20; i++)
  {
    if (danceOrder(0, i) >= 0)
    {
      total_num++;
      total_col += DanceSingle[danceOrder(0, i)].col;
    }
  }
  dance.init(total_col);
  int col_now = 0;
  for (int i = 0; i < total_num; i++)
  {
    dance.q.block(0, col_now, 6, DanceSingle[danceOrder(0, i)].col) = DanceSingle[danceOrder(0, i)].q.block(0, 0, 6, DanceSingle[danceOrder(0, i)].col);
    if (i > 0)
    {
      for (int j = 0; j < DanceSingle[danceOrder(0, i)].col; j++)
      {
        dance.t(0, col_now + j) = DanceSingle[danceOrder(0, i)].t(0, j) + dance.t(0, col_now - 1);
      }
    }
    else
    {
      dance.t.block(0, col_now, 1, DanceSingle[danceOrder(0, i)].col) = DanceSingle[danceOrder(0, i)].t.block(0, 0, 1, DanceSingle[danceOrder(0, i)].col);
    }

    col_now += DanceSingle[danceOrder(0, i)].col;
  }

  if (total_num == 0)
  {
    dance.init(60);
    dance.q << 0.000, 0.000, 0.000, 0.000, 0.032, 0.000, 0.000, 0.032, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
        0.000, 0.000, -0.070, -0.070, -0.049, -0.070, -0.070, -0.049, -0.070, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, -0.050, 0.000, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.040, -0.080, 0.000, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, -0.040, 0.080, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.800, -0.800, 0.800, 0.800, -0.800, -0.800, 0.800, 0.800, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, -1.000, 0.000, 1.000, 0.000, 0.000, 0.000,
        0.000, 0.000, -1.000, -1.000, -1.500, -1.000, -1.000, -1.500, -1.000, 0.000, 1.000, 1.000, 1.000, 1.000, 0.000, 0.000, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, -1.120, 0.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.200, -1.000, 0.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, -1.000, 0.000, 0.000,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.700, 0.700, -0.700, 0.700, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000, 0.700, -0.000, -0.700, -0.000, 0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.700, 0.000, -0.700, 0.000, 0.000, 0.000;
    dance.t << 1.00, 2.00, 2.50, 2.70, 3.70, 4.20, 4.40, 5.40, 6.40, 7.40, 7.90, 8.90, 9.40, 10.40, 10.90, 11.90, 12.40, 13.40, 14.40, 15.40, 16.40, 17.40, 18.40, 19.40, 20.40, 21.40, 22.40, 23.40, 24.40, 25.40, 26.40, 27.40, 28.40, 29.40, 30.40, 31.40, 32.40, 33.40, 34.40, 35.40, 36.40, 37.40, 38.40, 39.40, 40.40, 41.40, 42.40, 43.40, 44.40, 45.40, 46.40, 47.40, 48.40, 49.40, 50.40, 51.40, 52.40, 53.40, 54.40, 55.40;
  }
}
/// Add End

/// Add Begin by peibo, 2021-09-28, add reading of dance action dat file
template <typename T>
bool FSM_State_BalanceStand<T>::isDanceNumberEmpty(int danceNumber)
{
  auto iter = allDanceDataMap.find(danceNumber);
  return (iter == allDanceDataMap.end());
}

template <typename T>
void FSM_State_BalanceStand<T>::initDanceDataMap(std::string fileName)
{
  std::ifstream fileIn(fileName, ios::in);
  int dataNum = 0;
  if (fileIn.is_open())
  {
    while (!fileIn.eof())
    {
      string lineInfo;
      getline(fileIn, lineInfo);
      int index;
      char charBuffer[100];
      memset(charBuffer, 0, 100);
      if (2 == sscanf(lineInfo.c_str(), "%d %s", &index, charBuffer))
      {
        DanceCmd tempDanceCmd;
        QUADRUPED_INFO(_logger,"[FSM BalanceStand] Dance index: %d, file name: %s",index ,charBuffer);
        std::string tempStr;
        tempStr.append(THIS_COM "config/dance/");
        tempStr.append(charBuffer);
        if (getBalanceStandDanceData(tempStr, tempDanceCmd))
        {
          dataNum ++;
          allDanceDataMap.insert(std::make_pair(index,tempDanceCmd));
          QUADRUPED_INFO(_logger,"[FSM BalanceStand] Dance data acquisition succeeded.");
        }
        else
        {
          QUADRUPED_ERROR(_logger,"[FSM BalanceStand] Dance data acquisition failed.");
        }
      }
    }
  }
  else
  {
    QUADRUPED_ERROR(_logger,"dance file not exist!");
    fileIn.close();
  }
}
template <typename T>
bool FSM_State_BalanceStand<T>::getBalanceStandDanceData(std::string fileName, DanceCmd& outCmd)
{
  std::ifstream fileIn(fileName, ios::in | ios::binary);
  if (fileIn.is_open())
  {
    char getBZL[5];
    fileIn.read(getBZL, 5);
    std::string tempStr;
    tempStr.append(getBZL, getBZL + 5);
    if (tempStr == "BZL01")
    {
      unsigned int rows;
      unsigned int cols;
      fileIn.read((char *)(&rows), sizeof(rows));
      fileIn.read((char *)(&cols), sizeof(cols));
      long long preBufferSize = rows * cols * sizeof(double);
      long long curFileSize = fileIn.tellg();
      long long bufferSize;
      fileIn.seekg(0, ios::end);
      long long endSize = fileIn.tellg();
      bufferSize = endSize - curFileSize;
      if (rows == 7 && bufferSize == preBufferSize)
      {
        long bufferNum = rows * cols;
        double *bufferData = new double [bufferNum];
        fileIn.seekg(curFileSize);
        fileIn.read((char *)(bufferData), preBufferSize);
        if (fileIn.good())
        {
          outCmd.init(cols);
          copy(bufferData,bufferData + (6 * cols), outCmd.q.data());
          copy(bufferData + (6 * cols), bufferData + (7 * cols),outCmd.t.data());
          delete[] bufferData;
          fileIn.close();
          return true;
        }
        else
        {
          delete[] bufferData;
          fileIn.close();
        }
      }
    }
  }
  else
  {
    std::cout << "file not exist!" << std::endl;
    fileIn.close();
  }
  return false;
}
/// Add End

// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
