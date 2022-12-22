#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Locomotion : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData, 
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
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_Locomotion");
  // Keep track of the control iterations
  int iter = 0;
  ConvexMPCLocomotion* cMPCOld;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

  bool locomotionSafe();

  // Impedance control for the stance legs during locomotion
  void StanceLegImpedanceControl(int leg);
  /// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
  ControlFSMData<T>* _controlFSMData_ptr;
  /// Add End
  /// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
  void SafeChecker();
  /// Add End
};

#endif  // FSM_STATE_LOCOMOTION_H
