#ifndef FSM_STATE_RECOVERY_STANDUP_H
#define FSM_STATE_RECOVERY_STANDUP_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_RecoveryStand : public FSM_State<T> {
 public:
  FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData);

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
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_RecoveryStand");
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter = 0;
  int _flag = FoldLegs;

  // JPos
  Vec3<T> fold_jpos[4];
  Vec3<T> stand_jpos[4];
  Vec3<T> rolling_jpos[4];
  Vec3<T> initial_jpos[4];
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

  /// Mod Begin by wuchunming, 2021-03-17, mod rollover_ramp_iter
  const int rollover_ramp_iter = 135;
  /// origin code
  //const int rollover_ramp_iter = 150;
  /// Mod End

  const int rollover_settle_iter = 150;

  //const int fold_ramp_iter = 500;
  //const int fold_settle_iter = 500;
  const int fold_ramp_iter = 400;
  const int fold_settle_iter = 700;

  /// Mod Begin by anli, 20210609, use Quartic spline interpolation algorithm for StandUp Action, time 500x0.0002=1s
  const int standup_ramp_iter = 500;
  const int standup_settle_iter = 500;
  /// origin code
  //const int standup_ramp_iter = 250;
  //const int standup_settle_iter = 250;
  /// Mod End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  bool isNearStand = false;
  void CheckNearStand();
  /// Add End 
  
  /// Add Begin by peibo, 2021-08-04,modify the judgment conditions for the completion of the transition procedure for recovery stand mode
  bool isFinshStand = false;
  /// Add End 

  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

  /// Add Begin by anli,20210809, add adaptive rollover of 4 stages
  T start_roll_ang_;
  T curr_ang_;
  int *roll_leg_;
  int left_roll_leg_[4]{1,0,3,2};
  int right_roll_leg_[4]{0,1,2,3};
  enum SIDE{RIGHT = true, LEFT = false};
  bool origin_roll_side_ = SIDE::RIGHT;
  const int fast_fold_ramp_iter_ = 2000;
  bool fast_fold_flag_ = false;
  int ang_para_ = 1;
  enum ROLLOVER{MoveLegs = 3, PedalLegs, FoldAndUnFoldLegs, FoldAllLegs};
  
  Vec3<T> rollover_move_legs_jpos_[4];
  const int rollover_move_legs_ramp_iter_ = 1000;
  const int rollover_move_legs_settle_iter_ = 200;
  void RollOverMoveLegs(const int & iter);

  Vec3<T> rollover_pedal_legs_jpos_[4];
  const int rollover_pedal_legs_ramp_iter_ = 800;
  const int rollover_pedal_legs_settle_iter_ = 100;
  void RollOverPedalLegs(const int & iter);

  Vec3<T> rollover_foldunfold_legs_jpos_[4];
  const int rollover_foldunfold_legs_ramp_iter_ = 400;
  const int rollover_foldunfold_legs_settle_iter_ =200;
  void RollOverFoldAndUnFoldLegs(const int & iter);

  Vec3<T> rollover_foldall_legs_jpos_[4];
  const int rollover_foldall_legs_ramp_iter_ = 200;
  const int rollover_foldall_legs_settle_iter_ = 200;
  void RollOverFoldAllLegs(const int & iter);
  /// Add end
  /// Add Begin by wuchunming, 20211020, Anti kick function in recovery stand fsm state
  void SafeChecker(bool isFinshStand);
  /// Add End
};

#endif  // FSM_STATE_RECOVERY_STANDUP_H
