/*============================= Prone==============================*/
#ifndef FSM_STATE_PRONE_H
#define FSM_STATE_PRONE_H

#include "FSM_State.h"

/**
 * Robot from standing mode to lying down by peibo,2021-03-01
 */
template <typename T>
class FSM_State_Prone : public FSM_State<T> {
 public:
  FSM_State_Prone(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_Prone");
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int Prone = 1;
  static constexpr int FoldLegs = 2;
  static constexpr int RollOver = 3;
  /// Add Begin by peibo  2020-03-08,prone 2.0
  static constexpr int Spread = 4;
  static constexpr int FinalProne = 5;
  /// Add End

  unsigned long long _state_iter = 0;
  int _flag = StandUp;

  // JPos
  Vec3<T> fold_jpos[4];
  Vec3<T> stand_jpos[4];
  Vec3<T> rolling_jpos[4];
  Vec3<T> initial_jpos[4];
  /// Add Begin by peibo  2020-03-08,prone 2.0
  Vec3<T> prone_jpos[4];
  Vec3<T> fold_jposprone_jpos[4];
  Vec3<T> spread_jpos[4];
  Vec3<T> finalProne_jpos[4];
  /// Add End
  Vec3<T> zero_vec3;

  Vec3<T> f_ff;

  // iteration setup
  //const int rollover_ramp_iter = 300;
  //const int rollover_settle_iter = 300;

  //const int fold_ramp_iter = 1000;
  //const int fold_settle_iter = 1000;

  //const int standup_ramp_iter = 500;
  //const int standup_settle_iter = 500;

  // 0.5 kHz
  const int rollover_ramp_iter = 150;
  const int rollover_settle_iter = 150;

  //const int fold_ramp_iter = 500;
  //const int fold_settle_iter = 500;
  const int fold_ramp_iter = 400;
  const int fold_settle_iter = 700;

  const int standup_ramp_iter = 250;
  const int standup_settle_iter = 250;

  const int prone_ramp_iter = 500;
  const int prone_settle_iter = 250;

  const int spread_ramp_iter = 500;
  const int spread_settle_iter = 250;
  
  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _Prone(const int & iter);
  void _FoldLegs(const int & iter);
  void _Spread(const int & iter);
  void _FinalProne(const int & iter);

  bool _UpsideDown();
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

};

#endif  // FSM_STATE_PRONE_H
