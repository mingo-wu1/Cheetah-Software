/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_BackFlip.h"
#include <Utilities/Utilities_print.h>


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BackFlip<T>::FSM_State_BackFlip(ControlFSMData<T>* _controlFSMData)
  ///Mod by Peibo,2021-08-04,fix the wrong state name
    : FSM_State<T>(_controlFSMData, FSM_StateName::BACKFLIP, "BACKFLIP"){
  ///Ori Code:
  //: FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"){
  ///Mod End
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  zero_vec3.setZero();
  f_ff << 0.f, 0.f, -25.f;

  _data_reader = new DataReader(this->_data->_quadruped->_robotType, FSM_StateName::BACKFLIP);

  backflip_ctrl_ = new BackFlipCtrl<T>(_data_reader, this->_data->controlParameters->controller_dt);
  backflip_ctrl_->SetParameter();
  /// Add Begin by peibo,  2020-04-06, Add the preparation operation in the back flip mode 
  _isStartBackfilp = false;
  _cur_iter = 0;
  /// Add End
}


template <typename T>
void FSM_State_BackFlip<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
  _state_iter = 0;
  _count = 0;
  _curr_time = 0;
  _motion_start_iter = 0;
  _b_first_visit = true;
  
  // initial configuration, position
  for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }
  /// Add Begin by peibo,  2020-04-06, Add the preparation operation in the back flip mode 
  _isStartBackfilp = false;
  _cur_iter = 0;
  /// Add End

  /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
  _isSafeLanding = false;
  /// Add End
  this->transitionErrorMode = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BackFlip<T>::run() {
  /// Add  Begin by peibo zhaoyudong,  2020-04-06, Add the preparation operation in the back flip mode 
  //and extract the key parameters to the parameter table
	backflip_ctrl_->backfilp_tuck_iteration = this->_data->userParameters->backfilp_tuck_iteration;
	backflip_ctrl_->backfilp_ramp_end_iteration = this->_data->userParameters->backfilp_ramp_end_iteration;
	backflip_ctrl_->backfilp_tau_mult = this->_data->userParameters->backfilp_tau_mult;
	backflip_ctrl_->Kp_backfilp_step1 = this->_data->userParameters->Kp_backfilp_step1;
	backflip_ctrl_->Kd_backfilp_step1 = this->_data->userParameters->Kd_backfilp_step1;
	backflip_ctrl_->Kp_backfilp_step2_f = this->_data->userParameters->Kp_backfilp_step2_f;
	backflip_ctrl_->Kd_backfilp_step2_f = this->_data->userParameters->Kd_backfilp_step2_f;
	backflip_ctrl_->Kp_backfilp_step2_b = this->_data->userParameters->Kp_backfilp_step2_b;
	backflip_ctrl_->Kd_backfilp_step2_b = this->_data->userParameters->Kd_backfilp_step2_b;
	/// Add End
  /// Add Begin by peibo, zhaoyudong, 2021-05-13, the control torque coefficients of four legs in backflip mode are separated
	backflip_ctrl_->backfilp_tau_mult_fr = this->_data->userParameters->backfilp_tau_mult_fr;
	backflip_ctrl_->backfilp_tau_mult_fl = this->_data->userParameters->backfilp_tau_mult_fl;
	backflip_ctrl_->backfilp_tau_mult_br = this->_data->userParameters->backfilp_tau_mult_br;
	backflip_ctrl_->backfilp_tau_mult_bl = this->_data->userParameters->backfilp_tau_mult_bl;
	/// Add End
// Command Computation
  if (_b_running) {
    if (!_Initialization()) {
      ComputeCommand();
    }
  } else {
    _SafeCommand();
  }

  ++_count;
  _curr_time += this->_data->controlParameters->controller_dt;

}


template <typename T>
bool FSM_State_BackFlip<T>::_Initialization() { // do away with this?
  static bool test_initialized(false);
  if (!test_initialized) {
    test_initialized = true;
    QUADRUPED_INFO(_logger, "Test initialization is done");
  }
  if (_count < _waiting_count) {
    for (int leg = 0; leg < 4; ++leg) {
      this->_data->_legController->commands[leg].qDes = initial_jpos[leg];
      for (int jidx = 0; jidx < 3; ++jidx) {
        this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
        this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        this->_data->_legController->commands[leg].kpJoint(jidx,jidx) = 20.;
        this->_data->_legController->commands[leg].kdJoint(jidx,jidx) = 2.;
      }
    }
    return true;
  }
  
  return false;
}

template <typename T>
void FSM_State_BackFlip<T>::ComputeCommand() {
  if (_b_first_visit) {
    backflip_ctrl_->FirstVisit(_curr_time);
    _b_first_visit = false;
  }

  if(this->_data->controlParameters->use_rc){
    if(this->_data->_desiredStateCommand->rcCommand->mode == RC_mode::BACKFLIP_PRE){
      backflip_ctrl_->OneStep(_curr_time, true, this->_data->_legController->commands);
    }else{
      backflip_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
    }

  }else{
    /// Add Begin by peibo,  2020-04-06, Add the preparation operation in the back flip mode 
    if (this->_data->_desiredStateCommand->gamepadCommand->y == true)
    {
      if (++_cur_iter > _start_iter)
      {
        if(false == _isStartBackfilp){
          /// Add Begin by wuchunming 2021-05-27, add Finish Flag and fix backflip bug;
          _curr_time = 0;
          QUADRUPED_INFO(_logger, "now start backfilp");
        }
        _isStartBackfilp = true;
      }
    }
    else
    {
      _cur_iter = 0;
    }
    /// Add End
    /// Add Begin by peibo, 2020-09-06, add whether to turn on the delayed back flip flag and modify the back flip transition mode
    if(_isUseDalayBackflip == false || this->_data->autoTaskParam.backflipAutoTaskFlag == true)
      _isStartBackfilp = true;
    /// Add End
    /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
    _isSafeLanding = (false == _isStartBackfilp) ? true : false;
    /// Add End

    /// Mod Begin by peibo,  2020-04-06, Add the preparation operation in the back flip mode 
    //and extract the key parameters to the parameter table
    backflip_ctrl_->OneStep(_curr_time, _isStartBackfilp ? false : true, this->_data->_legController->commands);
    /// Ori Code:
    //backflip_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
    /// Mod End
  }

  if (backflip_ctrl_->EndOfPhase(this->_data->_legController->datas)) {
    /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
    _isSafeLanding = true;
    /// Add End
    /// Add Begin by peibo, 2020-09-06, add automatic task parameters
    if(this->_data->autoTaskParam.backflipAutoTaskFlag == true)
      this->_data->autoTaskParam.backflipAutoTaskFlag = false;
    /// Add End
    backflip_ctrl_->LastVisit();
  }
}

template <typename T>
void FSM_State_BackFlip<T>::_SafeCommand() {
  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
      this->_data->_legController->commands[leg].qDes[jidx] = this->_data->_legController->datas[leg].q[jidx];
      this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
    }
  }
}


template <typename T>
void FSM_State_BackFlip<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;

    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);

    //if(curr_iter == 0){ 
      //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n", 
        //_flag, curr_iter, _state_iter, _motion_start_iter); 
      //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
    //if(curr_iter == max_iter){ 
      //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n", 
        //_flag, curr_iter, _state_iter, _motion_start_iter); 
      //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BackFlip<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
  if(true == _isSafeLanding){/// Add End
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_BACKFLIP:
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_DAMP:
      this->nextStateName = FSM_StateName::DAMP;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    default:
      if (this->transitionErrorMode ==
        (uint8_t)this->_data->controlParameters->control_mode)
      {
        break;
      }
      this->transitionErrorMode = (uint8_t)this->_data->controlParameters->control_mode;
      QUADRUPED_WARN(_logger, "Bad Request: Cannot transition from %d to %d",
        K_BACKFLIP, (int)this->_data->controlParameters->control_mode);
  }
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_BackFlip<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::DAMP:  // normal
      this->transitionData.done = true;
      break;
      
    /// Mod Begin by peibo, 2021-09-06, add whether to turn on the delayed back flip flag and modify the back flip transition mode
    case FSM_StateName::BALANCE_STAND:

    case FSM_StateName::LOCOMOTION:

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = _isSafeLanding;
      if(_isSafeLanding == false)
      {
        run();
      }
      break;
    // Ori Code:
    // case FSM_StateName::BALANCE_STAND:
    //   this->transitionData.done = true;
    //   break;

    // case FSM_StateName::LOCOMOTION:
    //   this->transitionData.done = true;
    //   break;

    // case FSM_StateName::RECOVERY_STAND:
    //   this->transitionData.done = true;
    //   break;
    /// Mod End

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
void FSM_State_BackFlip<T>::onExit() {
  // nothing to clean up?
  /// Mod Begin by peibo, 2021-09-06, add whether to turn on the delayed back flip flag and modify the back flip transition mode
  // Ori Code:
  /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
  // _isSafeLanding = false;
  /// Add End
  /// Mod End
}

// template class FSM_State_BackFlip<double>;
template class FSM_State_BackFlip<float>;
