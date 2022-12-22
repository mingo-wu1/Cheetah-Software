/*============================= Prone==============================*/
/**
 * Robot from standing mode to lying down by peibo,2021-03-01
 */

#include "FSM_State_Prone.h"
#include <Utilities/Utilities_print.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Prone<T>::FSM_State_Prone(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PRONE, "PRONE")
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  zero_vec3.setZero();
  // goal configuration
  // Folding
  fold_jpos[0] << -0.3f, -1.8f, 2.7f;
  fold_jpos[1] << 0.3f, -1.8f, 2.7f;
  fold_jpos[2] << -0.3f, -1.8f, 2.7f;
  fold_jpos[3] << 0.3f, -1.8f, 2.7f;
  // Stand Up
  for (size_t i(0); i < 4; ++i)
  {
    //stand_jpos[i] << 0.f, -.9425f, 1.885f;
    stand_jpos[i] << 0.f, -.8f, 1.6f;
  }
  // Rolling
  rolling_jpos[0] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[1] << 1.3f, -3.1f, 2.77f;
  rolling_jpos[2] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[3] << 1.3f, -3.1f, 2.77f;

  f_ff << 0.f, 0.f, -25.f;
  /// Add Begin by  peibo  2021-03-08, add prone 2.0

  prone_jpos[0] << -0.0f, -1.4f, 2.7f;
  prone_jpos[1] << 0.0f, -1.4f, 2.7f;
  prone_jpos[2] << -0.0f, -1.4f, 2.7f;
  prone_jpos[3] << 0.0f, -1.4f, 2.7f;

  // spread_jpos[0] << -1.8f, 0.0f, 2.7f;
  // spread_jpos[1] << 1.8f, 0.0f, 2.7f;
  // spread_jpos[2] << -1.7f, 0.5f, 2.7f;
  // spread_jpos[3] << 1.7f, 0.5f, 2.7f;

  spread_jpos[0] << -0.52f, -1.00f, 2.7f;
  spread_jpos[1] << 0.52f, -1.00f, 2.7f;
  spread_jpos[2] << -0.52f, -1.00f, 2.7f;
  spread_jpos[3] << 0.52f, -1.00f, 2.7f;

  finalProne_jpos[0] << -0.52f, -1.00f, 2.7f;
  finalProne_jpos[1] << 0.52f, -1.00f, 2.7f;
  finalProne_jpos[2] << -0.52f, -1.00f, 2.7f;
  finalProne_jpos[3] << 0.52f, -1.00f, 2.7f;
  /// Add End
}

template <typename T>
void FSM_State_Prone<T>::onEnter()
{
  // Default is to not transition
  QUADRUPED_INFO(_logger, "START!!");
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
  _state_iter = 0;

  // initial configuration, position
  for (size_t i(0); i < 4; ++i)
  {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }

  T body_height =
      this->_data->_stateEstimator->getResult().position[2];

  _flag = StandUp;
  if (!_UpsideDown())
  { // Proper orientation
    ///Mod Begin by peibo,2021-05-08,Fix the jitter when you enter the prone mode
    if (body_height < 0.2 || body_height > 0.34)
    /// Ori Code:
    //if ((0.2 < body_height) && (body_height < 0.45))
    /// Mod End;
    {
      QUADRUPED_INFO(_logger, "body height is %f; Stand Up", body_height);
      _flag = StandUp;
    }
    else
    {
      QUADRUPED_INFO(_logger, "body height is %f; Go Prone", body_height);
      _flag = Prone;
    }
  }
  else
  {
    QUADRUPED_INFO(_logger, "UpsideDown (%d)", _UpsideDown());
  }
  _motion_start_iter = 0;
  this->transitionErrorMode = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Prone<T>::run()
{
  switch (_flag)
  {
  case StandUp:
    _StandUp(_state_iter - _motion_start_iter);
    break;
  case FoldLegs:
    _FoldLegs(_state_iter - _motion_start_iter);
    break;
  case RollOver:
    _RollOver(_state_iter - _motion_start_iter);
    break;
  case Prone:
    _Prone(_state_iter - _motion_start_iter);
    break;
  case Spread:
    _Spread(_state_iter - _motion_start_iter);
    break;
  case FinalProne:
    _FinalProne(_state_iter - _motion_start_iter);
    break;
  }

  ++_state_iter;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Prone<T>::checkTransition()
{

  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode)
  {
  /// Add Begin by  peibo  2021-03-01, add prone mode
  case K_PRONE:
    break;

  case K_RECOVERY_STAND:
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    break;

  /// Add Begin by peibo, 2021-08-04, ADD:transition from K_PRONE to K_PASSIVE
  case K_PASSIVE:  // normal c
    this->nextStateName = FSM_StateName::PASSIVE;
    break;
  /// Add End

  case K_DAMP:  // normal c
    this->nextStateName = FSM_StateName::DAMP;
    break;    


  default:
    if (this->transitionErrorMode ==
      (uint8_t)this->_data->controlParameters->control_mode)
    {
      break;
    }
    this->transitionErrorMode = (uint8_t)this->_data->controlParameters->control_mode;
    QUADRUPED_WARN(_logger, "Bad Request: Cannot transition from %d to %d",
      K_PRONE, (int)this->_data->controlParameters->control_mode);
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
TransitionData<T> FSM_State_Prone<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
  /// Add Begin by  peibo  2021-03-01, add prone mode
  case FSM_StateName::RECOVERY_STAND: // normal
    this->transitionData.done = true;
    break;
  /// Add End

  /// Add Begin by peibo, 2021-08-04, ADD:transition from K_PRONE to K_PASSIVE
  case FSM_StateName::PASSIVE:  // normal c
     this->transitionData.done = true;
     break;
  /// Add End

  case FSM_StateName::DAMP:  // normal c
     this->transitionData.done = true;
     break;

  default:
    QUADRUPED_WARN(_logger, "Something went wrong in transition");
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Prone<T>::onExit()
{
  // Nothing to clean up when exiting
}

template <typename T>
bool FSM_State_Prone<T>::_UpsideDown()
{
  //pretty_print(this->_data->_stateEstimator->getResult().rBody, std::cout, "Rot");
  //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
  if (this->_data->_stateEstimator->getResult().rBody(2, 2) < 0)
  {
    return true;
  }
  return false;
}

template <typename T>
void FSM_State_Prone<T>::_SetJPosInterPts(
    const size_t &curr_iter, size_t max_iter, int leg,
    const Vec3<T> &ini, const Vec3<T> &fin)
{

  float a(0.f);
  float b(1.f);

  // if we're done interpolating
  if (curr_iter <= max_iter)
  {
    b = (float)curr_iter / (float)max_iter;
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
template <typename T>
void FSM_State_Prone<T>::_RollOver(const int &curr_iter)
{

  for (size_t i(0); i < 4; ++i)
  {
    _SetJPosInterPts(curr_iter, rollover_ramp_iter, i,
                     initial_jpos[i], rolling_jpos[i]);
  }

  if (curr_iter > rollover_ramp_iter + rollover_settle_iter)
  {
    _flag = FoldLegs;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = rolling_jpos[i];
    _motion_start_iter = _state_iter + 1;
  }
}

template <typename T>
void FSM_State_Prone<T>::_StandUp(const int &curr_iter)
{
  T body_height = this->_data->_stateEstimator->getResult().position[2];
  bool something_wrong(false);

  if (_UpsideDown() || (body_height < 0.1))
  {
    something_wrong = true;
  }

  if ((curr_iter > floor(standup_ramp_iter * 0.7)) && something_wrong)
  {
    // If body height is too low because of some reason
    // even after the stand up motion is almost over
    // (Can happen when E-Stop is engaged in the middle of Other state)
    for (size_t i(0); i < 4; ++i)
    {
      initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    _flag = FoldLegs;
    _motion_start_iter = _state_iter + 1;

    QUADRUPED_WARN(_logger, "body height is still too low (%f) or UpsideDown (%d); Folding legs",
           body_height, _UpsideDown());
  }
  else
  {
    for (size_t leg(0); leg < 4; ++leg)
    {
      _SetJPosInterPts(curr_iter, standup_ramp_iter,
                       leg, initial_jpos[leg], stand_jpos[leg]);
    }
  }
  // feed forward mass of robot.
  //for(int i = 0; i < 4; i++)
  //this->_data->_legController->commands[i].forceFeedForward = f_ff;
  //Vec4<T> se_contactState(0.,0.,0.,0.);
  Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
  this->_data->_stateEstimator->setContactPhase(se_contactState);
  if (curr_iter > standup_ramp_iter + standup_settle_iter)
  {
    _flag = Prone;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = stand_jpos[i];
    _motion_start_iter = _state_iter + 1;
  }
}
/// Add Begin by  peibo  2021-03-01, add prone mode
template <typename T>
void FSM_State_Prone<T>::_Prone(const int &curr_iter)
{
  T body_height = this->_data->_stateEstimator->getResult().position[2];
  bool something_wrong(false);

  if (_UpsideDown() || (body_height > 0.2))
  {
    something_wrong = true;
  }

  if ((curr_iter > floor(prone_ramp_iter * 0.7)) && something_wrong)
  {
    // If body height is too low because of some reason
    // even after the stand up motion is almost over
    // (Can happen when E-Stop is engaged in the middle of Other state)
    for (size_t i(0); i < 4; ++i)
    {
      initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    _flag = StandUp;
    _motion_start_iter = _state_iter + 1;

    QUADRUPED_WARN(_logger, "body height is still too low (%f) or UpsideDown (%d); Folding legs",
           body_height, _UpsideDown());
  }
  else
  {
    for (size_t leg(0); leg < 4; ++leg)
    {
      _SetJPosInterPts(curr_iter, prone_ramp_iter,
                       leg, initial_jpos[leg], prone_jpos[leg]);
    }
  }
  // feed forward mass of robot.
  //for(int i = 0; i < 4; i++)
  //this->_data->_legController->commands[i].forceFeedForward = f_ff;
  //Vec4<T> se_contactState(0.,0.,0.,0.);
  Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
  this->_data->_stateEstimator->setContactPhase(se_contactState);
  if (curr_iter > prone_ramp_iter + prone_settle_iter)
  {
    _flag = FoldLegs;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = prone_jpos[i];
    _motion_start_iter = _state_iter + 1;
  }
}
/// Add End

/// Add Begin by  peibo  2021-03-01, add prone mode
template <typename T>
void FSM_State_Prone<T>::_FoldLegs(const int &curr_iter)
{

  for (size_t i(0); i < 4; ++i)
  {
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
                     initial_jpos[i], fold_jpos[i]);
  }
  if (curr_iter >= fold_ramp_iter + fold_settle_iter)
  {
#if (USE_LINKAGE_INDUSTRIAL == 0)
    _flag = Spread;
    //_flag = FinalProne;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = fold_jpos[i];
    _motion_start_iter = _state_iter + 1;
#else
    _flag = FinalProne;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = fold_jpos[i];
    _motion_start_iter = _state_iter + 1;
#endif
  }
}
/// Add End

/// Add Begin by  peibo  2021-03-01, add prone mode
template <typename T>
void FSM_State_Prone<T>::_Spread(const int &curr_iter)
{

  for (size_t i(0); i < 4; ++i)
  {
    _SetJPosInterPts(curr_iter, spread_ramp_iter, i,
                     initial_jpos[i], spread_jpos[i]);
  }
  if (curr_iter >= spread_ramp_iter + spread_settle_iter)
  {
    _flag = FinalProne;
    for (size_t i(0); i < 4; ++i)
      initial_jpos[i] = spread_jpos[i];
    _motion_start_iter = _state_iter + 1;
  }
}
/// Add End

/// Add Begin by  peibo  2021-03-01, add prone mode
template <typename T>
void FSM_State_Prone<T>::_FinalProne(const int &curr_iter)
{

  for (size_t i(0); i < 4; ++i)
  {
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
                     initial_jpos[i], finalProne_jpos[i]);
  }
  if (curr_iter >= fold_ramp_iter + fold_settle_iter)
  {
  }
}
/// Add End
//template class FSM_State_Prone<double>;
template class FSM_State_Prone<float>;
