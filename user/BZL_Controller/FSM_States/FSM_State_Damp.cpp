/*============================== Damp ==============================*/
/**
 * FSM State that calls only high damping factor. Meant to be a safe state where the
 * robot should not do anything as all commands other than kd will be set to 0.
 */

#include "FSM_State_Damp.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Damp<T>::FSM_State_Damp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::DAMP, "DAMP") {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Damp<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Damp<T>::run()
{
  // Do nothing, all commands should begin as zeros

  // std::cout<<"----RUN DAMP!----"<<std::endl;
  Mat3<T> Kd_soft_stop;
  Kd_soft_stop.setOnes();
  Kd_soft_stop = 5.0 * Kd_soft_stop;

  for (size_t leg = 0; leg < 4; leg++)
  {
    this->_data->_legController->commands[leg].tauFeedForward.setZero();
    this->_data->_legController->commands[leg].forceFeedForward.setZero();
    this->_data->_legController->commands[leg].qDes.setZero();
    this->_data->_legController->commands[leg].qdDes.setZero();
    this->_data->_legController->commands[leg].pDes.setZero();
    this->_data->_legController->commands[leg].vDes.setZero();
    this->_data->_legController->commands[leg].kpCartesian.setZero();
    this->_data->_legController->commands[leg].kdCartesian.setZero();
    this->_data->_legController->commands[leg].kpJoint.setZero();
    this->_data->_legController->commands[leg].kdJoint = Kd_soft_stop;
  }
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Damp<T>::testTransition() {
  this->transitionData.done = true;
  return this->transitionData;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Damp<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;
  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode)
  {

  case K_PASSIVE: // normal c (0)
    // Normal operation for state based transitions
    break;

  case K_DAMP:
    break;

  case K_RECOVERY_STAND:
    // Requested switch to joint PD control
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_DAMP << " to "
              << this->_data->controlParameters->control_mode << std::endl;
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
TransitionData<T> FSM_State_Damp<T>::transition()
{
  // Finish Transition

  switch (this->nextStateName)
  {

  case FSM_StateName::PASSIVE: // normal
    this->transitionData.done = true;
    break;

  case FSM_StateName::RECOVERY_STAND:
    this->transitionData.done = true;
    break;
  default:
    std::cout << "[CONTROL FSM] Something went wrong in transition"
              << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Damp<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_Passive<double>;
template class FSM_State_Damp<float>;
