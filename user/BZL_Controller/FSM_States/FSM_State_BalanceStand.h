#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

/// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
#include "Controllers/CubicSpline.h"
/// Add End

/// Add Begin by peibo, 2021-09-28, add reading of dance action dat file
#include <string>
#include <fstream>
#include <vector>
#include <map>
/// Add End

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
template<typename T>
struct TrajectoryPlanData{
  T plan_deltaT;
  T plan_time_step;
  T plan_body_pre_theta;
  T plan_body_des_theta;
  T gamepad_pre_theta;
  T gamepad_des_theta;
  void setZero(){
    plan_deltaT = plan_time_step = plan_body_pre_theta =
    plan_body_des_theta = gamepad_pre_theta = gamepad_des_theta = 0;
  }
};
/// Add End

/// Add Begin by anli, 2021-09-15, add automatic dance function
struct DanceCmd
{
  int col;
  Eigen::Matrix <float,6,Eigen::Dynamic> q;
  Eigen::Matrix <float,1,Eigen::Dynamic> t;
  void init(int col_num){
    col = col_num;
    q.resize(6,col);
    t.resize(1,col);
  };
};

struct DanceSinglCmd
{
  int col;
  Eigen::Matrix <float,6,8> q;
  Eigen::Matrix <float,1,8> t;
};
/// Add End

/**
 *
 */
template <typename T>
class FSM_State_BalanceStand : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData, 
	FSM_StateName stateNameIn, const std::string &stateStringIn);

  // Behavior to be carried out when entering a state
  BT::NodeStatus onStart() override final;

  // Run the normal behavior for the state
  BT::NodeStatus onRunning() override final;

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onHalted() override final;

 private:
  BZL_QUADRUPED::Logger _logger = BZL_QUADRUPED::Logger("FSM_State_BalanceStand");
  // Keep track of the control iterations
  int _iter = 0;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();

  /// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
  // Cubic Interpolation Method for Trajectory Plan
  T TrajectoryPlan(TrajectoryPlanData<T> &data, T _gamepad_des_theta, T _plan_deltaT);
  TrajectoryPlanData<T> r_TPData;
  TrajectoryPlanData<T> p_TPData;
  TrajectoryPlanData<T> y_TPData;
  /// Add End
  /// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
  SplineGeneratorData<T> splineGenData;
  CubicSpline<T> cs;
  int TPCount;
  void PlannerOfCubicSpline();
  void InterpolatorOfCubicSpline();
  /// Add End
  /// Add Begin by wuchunming, 2021-06-02, add body height offset
  double heightOffset = 0;
  /// Add End
  /// Add Begin by anli, 2021-09-15, add automatic dance function
  bool dance_num_change = false;
  bool first_gen_dance_single = true;
  int dance_num = -1; 
  Eigen::Matrix <int,1,20> danceOrder;
  DanceCmd dance;
  DanceSinglCmd DanceSingle[10];
  void danceCodeGen();
  //Add End

  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  T last_height_command = 0;

  Vec3<T> _ini_body_pos;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;
  /// Add Begin by peibo 2021-04-29,adding WBC operation separation in different modes
	ControlFSMData<T>* _controlFSMData_ptr;
	/// Add End

  /// Add Begin by peibo, 2021-09-28, add reading of dance action dat file
  std::map<int, DanceCmd> allDanceDataMap;
  bool isDanceNumberEmpty(int danceNumber);
  void initDanceDataMap(std::string fileName);
  bool getBalanceStandDanceData(std::string name, DanceCmd& outVec);
  /// Add End
};

#endif  // FSM_STATE_BALANCESTAND_H
