/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T>* _controlFSMData, 
	FSM_StateName stateNameIn, const std::string &stateStringIn)
    : FSM_State<T>(_controlFSMData, stateNameIn, stateStringIn) {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
BT::NodeStatus FSM_State_Passive<T>::onStart() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  return BT::NodeStatus::RUNNING;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
BT::NodeStatus FSM_State_Passive<T>::onRunning() {
  // Do nothing, all commands should begin as zeros
  testTransition();
    return BT::NodeStatus::RUNNING;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition() {
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
FSM_StateName FSM_State_Passive<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_PASSIVE:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case K_JOINT_PD:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;

    case K_STAND_UP:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    case K_RECOVERY_STAND:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    default:
      if (this->transitionErrorMode ==
        (uint8_t)this->_data->controlParameters->control_mode)
      {
        break;
      }
      this->transitionErrorMode = (uint8_t)this->_data->controlParameters->control_mode;
      QUADRUPED_WARN(_logger, "Bad Request: Cannot transition from %d to %d",
        K_PASSIVE, (int)this->_data->controlParameters->control_mode);
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
TransitionData<T> FSM_State_Passive<T>::transition() {
  // Finish Transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Passive<T>::onHalted() {
  // Nothing to clean up when exiting
}

// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;