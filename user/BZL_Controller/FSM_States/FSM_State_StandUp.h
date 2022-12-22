#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_StandUp : public FSM_State<T> {
 public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData, 
	FSM_StateName stateNameIn, const std::string &stateStringIn);

  // Behavior to be carried out when entering a state
  BT::NodeStatus onStart();

  // Run the normal behavior for the state
  BT::NodeStatus onRunning();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onHalted();

  TransitionData<T> testTransition();

 private:
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_StandUp");
  // Keep track of the control iterations
  int iter = 0;
  std::vector< Vec3<T> > _ini_foot_pos;
};

#endif  // FSM_STATE_STANDUP_H
