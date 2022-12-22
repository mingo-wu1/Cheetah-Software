/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_RecoveryStand.h"
#include <Utilities/Utilities_print.h>


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData, 
	FSM_StateName stateNameIn, const std::string &stateStringIn)
  ///Mod by Peibo,2021-08-04,fix the wrong state name
    : FSM_State<T>(_controlFSMData, stateNameIn, stateStringIn){
  ///Ori Code:
  //: FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"){
  ///Mod End"
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  zero_vec3.setZero();
  // goal configuration
  // Folding
  fold_jpos[0] << -0.0f, -1.4f, 2.7f;
  fold_jpos[1] << 0.0f, -1.4f, 2.7f;
  fold_jpos[2] << -0.0f, -1.4f, 2.7f;
  fold_jpos[3] << 0.0f, -1.4f, 2.7f;
  // Stand Up
  for(size_t i(0); i<4; ++i){
    //stand_jpos[i] << 0.f, -.9425f, 1.885f;
    stand_jpos[i] << 0.f, -.8f, 1.6f;
  }
   
  /// Mod Begin by anli, 20210809, new_rollover mode
  rollover_move_legs_jpos_[0] << -0.43f, -1.7f, 2.7f;
  rollover_move_legs_jpos_[1] << 0.43f, 2.1f, 1.5f;
  rollover_move_legs_jpos_[2] << 0.43f, -2.f, 2.7f;
  rollover_move_legs_jpos_[3] << 0.43f, 2.1f, 1.5f;

  rollover_pedal_legs_jpos_[0] << -0.43f, -2.9f, 2.f;
  rollover_pedal_legs_jpos_[1] << 0.43f, 2.7f, 1.0f;
  rollover_pedal_legs_jpos_[2] << -0.43f, -2.7f, 2.f;
  rollover_pedal_legs_jpos_[3] << 0.43f, 2.7f, 1.f;

  rollover_foldunfold_legs_jpos_[0] << 0.f, -2.9f, 2.f;
  rollover_foldunfold_legs_jpos_[1] << -0.f, -1.5f, 2.6f;
  rollover_foldunfold_legs_jpos_[2] << 0.f, -3.f, 2.6f;
  rollover_foldunfold_legs_jpos_[3] << -0.f, -1.5f, 2.6f;

  rollover_foldall_legs_jpos_[0] << 0.43f, -2.9f, 2.6f;
  rollover_foldall_legs_jpos_[1] << -0.43f, -1.5f, 2.6f;
  rollover_foldall_legs_jpos_[2] << 0.43f, -1.4f, 2.6f;
  rollover_foldall_legs_jpos_[3] << -0.43f, -1.5f, 2.6f;

  for (int leg = 0; leg < 4; ++leg)
  {
    rollover_pedal_legs_jpos_[leg] = rollover_pedal_legs_jpos_[leg] - rollover_move_legs_jpos_[leg];
    rollover_move_legs_jpos_[leg] = rollover_move_legs_jpos_[leg] - fold_jpos[leg];
  }
  /// origin code
  // Rolling
  /*
  rolling_jpos[0] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[1] << 1.3f, -3.1f, 2.77f;
  rolling_jpos[2] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[3] << 1.3f, -3.1f, 2.77f;
  */
  /// Mod End
  f_ff << 0.f, 0.f, -25.f;

  		
  /// Add Begin by peibo, 2021-08-04,modify the judgment conditions for the completion of the transition procedure for recovery stand mode
  isFinshStand = false;
  /// Add End 
}

template <typename T>
BT::NodeStatus FSM_State_RecoveryStand<T>::onStart() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
  _state_iter = 0;
  
  // initial configuration, position
  for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }

  T body_height = 
    this->_data->_stateEstimator->getResult().position[2];

  _flag = FoldLegs;
  if( !_UpsideDown() ) { // Proper orientation
    if (  (0.2 < body_height) && (body_height < 0.45) ){
      QUADRUPED_INFO(_logger, "body height is %f; Stand Up", body_height);
      _flag = StandUp;
    }else{
      QUADRUPED_INFO(_logger, "body height is %f; Folding legs", body_height);
    }
  }else{
      QUADRUPED_INFO(_logger, "UpsideDown (%d)", _UpsideDown() );
  }
  _motion_start_iter = 0;

  CheckNearStand();

  /// Add Begin by peibo, 2021-08-04,modify the judgment conditions for the completion of the transition procedure for recovery stand mode
  isFinshStand = false;
  /// Add End 

  return BT::NodeStatus::RUNNING;
}

template <typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown(){
  //pretty_print(this->_data->_stateEstimator->getResult().rBody, std::cout, "Rot");
  //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
  if(this->_data->_stateEstimator->getResult().rBody(2,2) < 0){
    return true;
  }
  return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
BT::NodeStatus FSM_State_RecoveryStand<T>::onRunning() {
  /// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
  SafeChecker(isFinshStand);
  /// Add End

  switch(_flag){
    case StandUp:
      _StandUp(_state_iter - _motion_start_iter);
      break;
    case FoldLegs:
      _FoldLegs(_state_iter - _motion_start_iter);
      break;
    case RollOver:
      _RollOver(_state_iter - _motion_start_iter);
      break;

    /// add begin by anli  20210809, add adaptive rollover of 4 stages
    case ROLLOVER::MoveLegs:
      RollOverMoveLegs(_state_iter - _motion_start_iter); // move legs to destination position
      break;
    case ROLLOVER::PedalLegs:
      RollOverPedalLegs(_state_iter - _motion_start_iter); // pedal legs to roll body
      break;
    case ROLLOVER::FoldAndUnFoldLegs:
      RollOverFoldAndUnFoldLegs(_state_iter - _motion_start_iter); // fold side legs and unfold side legs to land
      break;
    case ROLLOVER::FoldAllLegs:
      RollOverFoldAllLegs(_state_iter - _motion_start_iter); // fold all legs
      break;
    /// add end
  }

 ++_state_iter;

 return BT::NodeStatus::RUNNING;
}

template <typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin){

    float a(0.f);
    float b(1.f);

    /// Add Begin by anli, 20210609, use Quartic spline interpolation algorithm for StandUp Action
    float c(1.f);

    if(curr_iter <= max_iter) 
    {
      b = static_cast<float>(curr_iter) / static_cast<float>(max_iter);
      switch(_flag)
      {
        case StandUp:
          c = static_cast<float>(curr_iter) / static_cast<float>(max_iter);
          b = 3. * c * c * c * c - 8. * c * c * c + 6. * c * c;
          break;
        default:
        /// Mod begin by anli, 20210820, change
          b = static_cast<float>(curr_iter) / static_cast<float>(max_iter);
        /// orgin code
        /*
        case FoldLegs || RollOver:
          b = (float)curr_iter/(float)max_iter;
          break;
        */
        /// Mod End
      }
      a = 1.f - b;
    }

    /// origin code
    //// if we're done interpolating
    //if(curr_iter <= max_iter) {
    //  b = (float)curr_iter/(float)max_iter;
    //  a = 1.f - b;
    //}    
    /// Mod End

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

template <typename T>
void FSM_State_RecoveryStand<T>::_RollOver(const int & curr_iter){

  for(size_t i(0); i<4; ++i){
    _SetJPosInterPts(curr_iter, rollover_ramp_iter, i, 
        initial_jpos[i], rolling_jpos[i]);
  }

  if(curr_iter > rollover_ramp_iter + rollover_settle_iter){
    _flag = FoldLegs;
    for(size_t i(0); i<4; ++i) initial_jpos[i] = rolling_jpos[i];
    _motion_start_iter = _state_iter+1;
  }
}

template <typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int & curr_iter){
  T body_height = this->_data->_stateEstimator->getResult().position[2];
  bool something_wrong(false);

  if( _UpsideDown() || (body_height < 0.1 ) ) { 
    something_wrong = true;
    isNearStand = false;
  }

  if( (curr_iter > floor(standup_ramp_iter*0.7) ) && something_wrong){
    // If body height is too low because of some reason 
    // even after the stand up motion is almost over 
    // (Can happen when E-Stop is engaged in the middle of Other state)
    for(size_t i(0); i < 4; ++i) {
      initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    _flag = FoldLegs;
    _motion_start_iter = _state_iter+1;

    QUADRUPED_WARN(_logger, "body height is still too low (%f) or UpsideDown (%d); Folding legs", 
        body_height, _UpsideDown() );

  }else{
    for(size_t leg(0); leg<4; ++leg){
      /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      if(isNearStand) initial_jpos[leg] = stand_jpos[leg];
      /// Add End 
      _SetJPosInterPts(curr_iter, standup_ramp_iter, 
          leg, initial_jpos[leg], stand_jpos[leg]);
      /// Ori Code:
      // _SetJPosInterPts(curr_iter, standup_ramp_iter, 
      //     leg, initial_jpos[leg], stand_jpos[leg]);
      /// Mod End 
    }
    /// Add Begin by peibo, 2021-08-04,modify the judgment conditions for the completion of the transition procedure for recovery stand mode
    if(isNearStand || curr_iter >= standup_ramp_iter)
      isFinshStand = true;
    /// Add End 
  }
  // feed forward mass of robot.
  //for(int i = 0; i < 4; i++)
  //this->_data->_legController->commands[i].forceFeedForward = f_ff;
  //Vec4<T> se_contactState(0.,0.,0.,0.);
  Vec4<T> se_contactState(0.5,0.5,0.5,0.5);
  this->_data->_stateEstimator->setContactPhase(se_contactState);

}

/// Mod Begin by anli, 20210809, add adaptive rollover of 4 stages
template <typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int & curr_iter){
  // set curr joint pos to des joint pos
  isNearStand = false;
  for(size_t leg(0); leg < 4; ++leg)
  {
    int ramp_iter = fast_fold_flag_ ? fast_fold_ramp_iter_ : fold_ramp_iter;
    _SetJPosInterPts(curr_iter, ramp_iter, leg, initial_jpos[leg], fold_jpos[leg]);
  }

  // set upsidedown action joint pos by roll des
  if(curr_iter >= fold_ramp_iter + fold_settle_iter)
  {
    fast_fold_flag_ = false;
    if(_UpsideDown()){
      _flag = ROLLOVER::MoveLegs;
      if((this->_data->_stateEstimator->getResult().rpy[0] < 0) && (this->_data->_stateEstimator->getResult().rpy[0] > -3))
      {
        roll_leg_ = left_roll_leg_;
        ang_para_ = -1;
      }
      else
      {
        roll_leg_ = right_roll_leg_;
        ang_para_ = 1;
      }
      for (size_t leg(0); leg < 4; leg++)
      {
        rollover_move_legs_jpos_[leg][0] = ang_para_ * rollover_move_legs_jpos_[leg][0];
        rollover_pedal_legs_jpos_[leg][0] = ang_para_ * rollover_pedal_legs_jpos_[leg][0];
        rollover_foldunfold_legs_jpos_[leg][0] = ang_para_ * rollover_foldunfold_legs_jpos_[leg][0];
        rollover_foldall_legs_jpos_[leg][0] = ang_para_ * rollover_foldall_legs_jpos_[leg][0];
      }
    }
    else
    {
      _flag = StandUp;
    }
    for(size_t leg(0); leg < 4; leg++)
      initial_jpos[leg] = fold_jpos[leg];
    _motion_start_iter = _state_iter + 1;
  }
}

/// origin code
/*
template <typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int & curr_iter){

  for(size_t i(0); i<4; ++i){
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i, 
        initial_jpos[i], fold_jpos[i]);
  }
  if(curr_iter >= fold_ramp_iter + fold_settle_iter){
    if(_UpsideDown()){
      _flag = RollOver;
      for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
    }else{
      _flag = StandUp;
      for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
    }
    _motion_start_iter = _state_iter + 1;
  }
}
*/
/// Mod End

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_RECOVERY_STAND:
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_BACKFLIP: 
      this->nextStateName = FSM_StateName::BACKFLIP;
      break;

    case K_FRONTJUMP: 
      this->nextStateName = FSM_StateName::FRONTJUMP;
      break;

    case K_VISION: 
      this->nextStateName = FSM_StateName::VISION;
      break;

    /// Add Begin by peibo, 2021-03-01, add prone mode
    case K_PRONE:
      this->nextStateName = FSM_StateName::PRONE;
      break;
    /// Add End

    default:
      if (this->transitionErrorMode ==
        (uint8_t)this->_data->controlParameters->control_mode)
      {
        break;
      }
      this->transitionErrorMode = (uint8_t)this->_data->controlParameters->control_mode;
      QUADRUPED_WARN(_logger, "Bad Request: Cannot transition from %d to %d",
        K_RECOVERY_STAND, (int)this->_data->controlParameters->control_mode);
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
TransitionData<T> FSM_State_RecoveryStand<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    /// Mod Begin by peibo, 2021-08-04,modify the judgment conditions for the completion of the transition procedure for recovery stand mode
    case FSM_StateName::BALANCE_STAND:
 
    case FSM_StateName::LOCOMOTION:
 
    case FSM_StateName::BACKFLIP:
 
    case FSM_StateName::FRONTJUMP:
 
    case FSM_StateName::VISION:
    
    case FSM_StateName::PRONE:
      this->transitionData.done = isFinshStand;
      if(this->transitionData.done == false)
      {
        // this->run();
      }
      break;
    /// Ori Code:
    // case FSM_StateName::BALANCE_STAND:
    //   this->transitionData.done = true;
    //   break;
    // case FSM_StateName::LOCOMOTION:
    //   this->transitionData.done = true;
    //   break;
    // case FSM_StateName::BACKFLIP:
    //   this->transitionData.done = true;
    //   break;
    // case FSM_StateName::FRONTJUMP:
    //   this->transitionData.done = true;
    //   break;
    // case FSM_StateName::VISION:
    //   this->transitionData.done = true;
    //   break;
    // /// Add Begin by peibo, 2021-03-01, add prone mode
    // case FSM_StateName::PRONE:
    //   this->transitionData.done = true;
    //   break;
    // /// Add End
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
void FSM_State_RecoveryStand<T>::onHalted() {
  // Nothing to clean up when exiting
}

/// Add Begin by anli, 20210809, add adaptive rollover of 4 stages
template <typename T>
void FSM_State_RecoveryStand<T>::CheckNearStand() {
  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  if(_flag == StandUp)
  {
    isNearStand = true;
    for(int leg(0); leg < 4; ++leg)
      for(int joint(0); joint < 3; ++joint)
        if(fabs(this->_data->_legController->datas[leg].q[joint] - stand_jpos[leg][joint]) > 5.0 / 180.0 * M_PI)
        {
          isNearStand = false;
          break;
        }
  }
  else
    isNearStand = false;
  /// Add End
}

template <typename T>
void FSM_State_RecoveryStand<T>::RollOverMoveLegs(const int& curr_iter){
  // get imu rpy result
  if(curr_iter == 1){
    start_roll_ang_ = this->_data->_stateEstimator->getResult().rpy[0];
  }
  curr_ang_ = this->_data->_stateEstimator->getResult().rpy[0];
  
  // set curr joint pos to des joint pos
  for(size_t leg(0); leg < 4; ++leg){
    _SetJPosInterPts(curr_iter, rollover_move_legs_ramp_iter_, leg, 
      initial_jpos[leg], initial_jpos[leg] + rollover_move_legs_jpos_[*(roll_leg_ + leg)]);
  }

  // init next action joint pos by iter or roll
  if((curr_iter > rollover_move_legs_ramp_iter_ + rollover_move_legs_settle_iter_)
	|| ((abs(curr_ang_ - start_roll_ang_) > 0.1) && (abs(curr_ang_) < 1.95))){
    _flag = ROLLOVER::PedalLegs;
    for(size_t leg(0); leg < 4; ++leg)
      initial_jpos[leg] = this->_data->_legController->datas[leg].q;
    _motion_start_iter = _state_iter + 1;
  }
}

template <typename T>
void FSM_State_RecoveryStand<T>::RollOverPedalLegs(const int& curr_iter){
  // get imu rpy result
  if(curr_iter == 1){
    start_roll_ang_ = this->_data->_stateEstimator->getResult().rpy[0];
  }
  curr_ang_ = this->_data->_stateEstimator->getResult().rpy[0];

  // set curr joint pos to des joint pos
  for(size_t leg(0); leg<4; ++leg){
    _SetJPosInterPts(curr_iter, rollover_pedal_legs_ramp_iter_, leg, 
      initial_jpos[leg], initial_jpos[leg] + rollover_pedal_legs_jpos_[*(roll_leg_ + leg)]);
  }
    
  // init next action joint pos by iter or roll
  if((abs(curr_ang_) < 1.6) || 
    curr_iter > rollover_pedal_legs_ramp_iter_ + rollover_pedal_legs_settle_iter_)
  {
    _flag = ROLLOVER::FoldAndUnFoldLegs;
    for(size_t leg(0); leg<4; ++leg) 
      initial_jpos[leg] =this->_data->_legController->datas[leg].q;
    _motion_start_iter = _state_iter + 1;
  }
}

template <typename T>
void FSM_State_RecoveryStand<T>::RollOverFoldAndUnFoldLegs(const int& curr_iter){
  // set curr joint pos to des joint pos
  for(size_t j(0); j < 2; ++j)
  {
    _SetJPosInterPts(curr_iter, rollover_foldunfold_legs_ramp_iter_, 2 * j,
      initial_jpos[2 * j], rollover_foldunfold_legs_jpos_[*(roll_leg_ + 2 * j)]);
    _SetJPosInterPts(curr_iter, 0.7 * rollover_foldunfold_legs_ramp_iter_, 2 * j + 1,
      initial_jpos[2 * j + 1], rollover_foldunfold_legs_jpos_[*(roll_leg_ + 2 * j + 1)]);
  }

  // init next action joint pos by iter or roll
  if((curr_iter > rollover_foldunfold_legs_ramp_iter_ + rollover_foldunfold_legs_settle_iter_)
      || (abs(this->_data->_stateEstimator->getResult().rpy[0])<1.5))
  {
    _flag = ROLLOVER::FoldAllLegs;
    for(size_t leg(0); leg < 4; ++leg)
      initial_jpos[leg] = this->_data->_legController->datas[leg].q;
    _motion_start_iter = _state_iter + 1;
  }  
}

template <typename T>
void FSM_State_RecoveryStand<T>::RollOverFoldAllLegs(const int& curr_iter){
  // set curr joint pos to des joint pos
  for(size_t leg(0); leg < 4; ++leg)
  {
    _SetJPosInterPts(curr_iter, rollover_foldall_legs_ramp_iter_, leg,
      initial_jpos[leg], rollover_foldall_legs_jpos_[*(roll_leg_ + leg)]);
  }

  // init next action joint pos by iter
  if((curr_iter > rollover_foldall_legs_ramp_iter_))
  {
    _flag = FoldLegs;
    for(size_t i(0); i < 4; ++i)
      initial_jpos[i] = this->_data->_legController->datas[i].q;
    _motion_start_iter = _state_iter + 1;
    for (int leg = 0; leg < 4; ++leg)
    {
      rollover_move_legs_jpos_[leg][0] = ang_para_ * rollover_move_legs_jpos_[leg][0];
      rollover_pedal_legs_jpos_[leg][0] = ang_para_ * rollover_pedal_legs_jpos_[leg][0];
      rollover_foldunfold_legs_jpos_[leg][0] = ang_para_ * rollover_foldunfold_legs_jpos_[leg][0];
      rollover_foldall_legs_jpos_[leg][0] = ang_para_ * rollover_foldall_legs_jpos_[leg][0];
    }
  }
}

/// add end 

/// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
template <typename T>
void FSM_State_RecoveryStand<T>::SafeChecker(bool isFinshStand){
  static int timer = 0;
  if(isFinshStand){
    auto data = this->_data->_stateEstimator->getResult();
    if(std::abs(data.vBody[0]) >= 0.2 || std::abs(data.vBody[1]) >= 0.2){ 
      if(++timer >= 1 / (this->_data->controlParameters->controller_dt * 20)){ // 0.2s
        this->_data->_desiredStateCommand->rcCommand->mode = RC_mode::LOCOMOTION;
      	timer = 0;
      }
    }else{
      timer = 0;
    }
  }else{}
}
/// Add End

// template class FSM_State_RecoveryStand<double>;
template class FSM_State_RecoveryStand<float>;
