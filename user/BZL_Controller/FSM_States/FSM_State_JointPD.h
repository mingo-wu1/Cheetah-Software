#ifndef FSM_STATE_JOINTPD_H
#define FSM_STATE_JOINTPD_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_JointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_JointPD(ControlFSMData<T>* _controlFSMData, 
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

 private:
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_JointPD");
  // Keep track of the control iterations
  int iter = 0;
  DVec<T> _ini_jpos;
};

#endif  // FSM_STATE_JOINTPD_H
