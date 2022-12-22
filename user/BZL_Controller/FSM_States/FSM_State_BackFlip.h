#ifndef FSM_STATE_BACKFLIP_H
#define FSM_STATE_BACKFLIP_H

#include "FSM_State.h"
#include <Controllers/BackFlip/DataReader.hpp>
#include <Controllers/BackFlip/BackFlipCtrl.hpp>


/**
 *
 */
template <typename T>
class FSM_State_BackFlip : public FSM_State<T> {
 public:
  FSM_State_BackFlip(ControlFSMData<T>* _controlFSMData, 
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
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_BackFlip");
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int Preparation = 0;
  static constexpr int Flip = 1;
  static constexpr int Landing = 2;

  /// Add Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
  bool _isSafeLanding = false;
  /// Add End  

  unsigned long long _state_iter;
  int _flag = Preparation;

  // JPos
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;
  Vec3<T> f_ff;
  
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

  DataReader* _data_reader;
  bool _b_running = true;
  bool _b_first_visit = true;
  int _count = 0;
  int _waiting_count = 6;
  float _curr_time = 0;
  BackFlipCtrl<T>* backflip_ctrl_;

  void SetTestParameter(const std::string& test_file);
  bool _Initialization();
  void ComputeCommand();
  void _SafeCommand();

  /// Add Begin by peibo, 2020-04-06, Add the preparation operation in the back flip mode
  bool _isStartBackfilp = false;
  int _start_iter = 1500;
  int _cur_iter = 0;
  /// Add End
  /// Add Begin by peibo, 2020-09-06, Add whether to turn on the delayed back flip flag and modify the back flip transition mode
  bool _isUseDalayBackflip = false;
  /// Add End

};

#endif  // FSM_STATE_BACKFLIP_H