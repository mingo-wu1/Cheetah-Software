#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include <SparseCMPC/SparseCMPC.h>
#include "cppTypes.h"
#include "Gait.h"
#include "Logger/Logger.h"

#include <cstdio>

/// Add Begin by wuchunming, 2020-03-13, log pDes,vDes,p,v
#include <fstream>
/// Add End

/// Add Begin by wuchunming, 20210703
#include <FSM_States/FSM_State_BalanceStand.h>
/// Add End

using Eigen::Array4f;
using Eigen::Array4i;

/*CMPC结果变量*/
template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];                                                        //四条腿的反作用力
  Vec4<T> contactPhase;                                                                       //接触支撑腿的状态
};

/*凸MPC跳跃*/
struct CMPC_Jump {
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg) {
    (void)seg;
    //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger) {
    (void)seg;
    if(!pressed && trigger) {
      if(!jump_pending && !jump_in_progress) {
        jump_pending = true;
        //printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg) {
    debug(seg);

    if(jump_pending && seg == START_SEG) {
      jump_pending = false;
      jump_in_progress = true;
      //printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if(jump_in_progress) {
      if(seg == END_SEG && seg != last_seg_seen) {
        seen_end_count++;
        if(seen_end_count == END_COUNT) {
          seen_end_count = 0;
          jump_in_progress = false;
          //printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};

/*凸MPC运动*/
class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, BZL_UserParameters* parameters);

  void Initialize();                                                                                              //（1）初始化MPC函数
  void SetParams(BZL_UserParameters* parameters);

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  bool Transition();
  /// Add End 

  template<typename T>
  void Run(ControlFSMData<T>& data);                                                                              //（2）MPC运行过程函数（重点）
  bool currently_jumping_ = false;

  Vec3<float> pos_body_des;
  Vec3<float> vel_body_des;
  Vec3<float> acc_body_des;

  Vec3<float> pos_body_rpy_des;
  Vec3<float> vel_body_rpy_des;

  Vec3<float> pos_foot_world_des[4];                                                                              // 足端期望 足端轨迹跟踪用
  Vec3<float> vel_foot_world_des[4];
  Vec3<float> acc_foot_world_des[4];

  Vec3<float> rforce_des[4];

  Vec4<float> contact_state;

private:
  BZL_QUADRUPED::Logger _logger;
  void SetupCommand(ControlFSMData<float> & data);

  float yaw_turn_rate_;                                                                                         // 期望 机体下
  float yaw_des_;

  float roll_des_;
  float pitch_des_;

  float x_vel_des_ = 0.;
  float y_vel_des_ = 0.;

  // High speed running
  //float _body_height = 0.34;
  float body_height_ = 0.29;
  float body_height_jumping_ = 0.36;

  void RecomputeTiming(int iterations_per_mpc);                                                                 //（3）重新计算时间函数
  void UpdateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);                            //（4）更新MPC数据函数
  void SolveDenseMPC(int *mpcTable, ControlFSMData<float> &data);                                               //（5）解稠密MPC函数
  void SolveSparseMPC(int *mpcTable, ControlFSMData<float> &data);                                              //（6）解稀疏MPC函数
  void InitSparseMPC();                                                                                         //（7）初始化稀疏MPC函数
  int iters_between_mpc_;                                                                                       //迭代次数在mpc之间
  int horizon_length_;                                                                                          //mpc分段数，即预测多少个mpc周期长的输入
  int default_iters_between_mpc_;                                                                               //默认迭代次数在mpc之间
  float dt_;                                                                                                    //一般频率下时间间隔
  
  /// Add Begin by zhaobo, lihao, 2020-03-03, add BZL parameters, mod mpc algorithm
  int iters_between_gait_seg_;
  int gait_seg_;
  float foot_height_;
  float dt_gait_seg_;
  float raibert_coeff_;
  float mu_;
  float rforce_max_;
  float alpha_;
  /// Add End
  /// Add Begin by wuchunming, 2021-04-23, add select for selecting foot trajectory plan, enum for selecting
  enum class FootTrajectoryPlanType { PARABOLA, CUBICSPLINE };
  FootTrajectoryPlanType foot_traj_plan_type_;
  /// Add End

  float dt_mpc_;                                                                                                //mpc运算周期
  int iter_counter_ = 0;                                                                                        //频率控制计数器
  Vec3<float> f_fforce_[4];                                                                                     //四脚力输出
  Vec4<float> swing_times_;                                                                                     //摆动时间
  FootSwingTrajectory<float> foot_swing_traj_[4];
  /// Mod Begin by  zhaoyudong peibo, 2021-02-25, add triangle gait 
  OffsetDurationGait trotting_, bounding_, pronking_, jumping_, galloping_, standing_, trot_running_, walking_, walking2_, pacing_\
  ,triangle1243_,triangle1234_,triangle1423_,triangle1324_,trotting_stair_;
  /// Ori Code:
  // OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  /// Mod End
  MixedFrequncyGait random_, random2_;
  Mat3<float> Kp_, Kd_, Kp_stance_, Kd_stance_;
  bool first_run_ = true;                                                                                       //首次运行
  bool first_swing_[4];                                                                                         //首次摆动
  float swing_time_remaining_[4];                                                                               //剩余摆动时间
  float stand_traj_[6];
  int current_gait_;                                                                                            //当前步态
  int gait_number_;                                                                                             //步态

  Vec3<float> world_pos_des_;                                                                                   //机体期望位置
  Vec3<float> rpy_integral_;                                                                                    //初始欧拉角
  Vec3<float> rpy_compensate_;
  float x_comp_integral_ = 0;
  Vec3<float> pos_foot_world_[4];                                                                               //四足端坐标
  float trajAll_[12*36];                                                                                        //mpc格式储存轨迹

  BZL_UserParameters* params_ = nullptr;
  CMPC_Jump jump_state_;

  vectorAligned<Vec12<double>> sparse_traj_;

  SparseCMPC sparse_cmpc_;

  /// Add Begin by yuzhiyou, wuchunming, 2021-04-06, add updateBodyPitch for Calculate the inclination of the body to the ground
  float updateBodyPitch(ControlFSMData<float>& data);
  /// Add End 
  /// Add Begin by peibo, 2021-04-09,Preprocessing speed and gait
  int speed_Gait_Processing(ControlFSMData<float>& data,int gaitNumber);
  /// Add End 
  /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
  void kpkd_parameter_adjustment_Processing(ControlFSMData<float>& data,int gaitNumber);
  /// Add End 
  /// Add Begin by peibo zhaoyudong, 2021-04-09,Velocity expected smoothing
  float velocityExpectedSmoothing(bool use_smooth,float speed_step,float filter,float vel_des,float vel_source);
  /// Add End
  /// Add Begin by peibo,wangjie, 2021-05-28,optimize anti kick function
  void antiKickOptimize(int legIndex,float& pfX,float& pfY,const StateEstimate<float>& seResult);
  /// Add End
  /// Add Begin by yuzhiyou, peibo, 2021-04-23, Modify the calculation of landing point
  Vec4<float> pos_xfoot_stance_;
  Vec4<float> pos_zfoot_stance_;
   /// Add End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  bool startTransition = false;
  /// Add End 

  /// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion 
  enum RPYMODE_{YAW, YAWPITCH, ROLLPITCH, YAWROLL, count};
  int rpy_mode_ = 0;
  bool allow_rpy_mode_switch_ = false;

  TrajectoryPlanData<float> yp_data1_;
  TrajectoryPlanData<float> rp_data2_;
  TrajectoryPlanData<float> rp_data3_;
  TrajectoryPlanData<float> yr_data4_;
  float TrajectoryPlan(TrajectoryPlanData<float> &data, float _gamepad_des_theta, float _plan_deltaT);
  float pitch_turn_rate_ = 0.;
  float roll_turn_rate_ = 0.;
  float height_offset_ = 0.;
  void footPos();

  bool allow_lateral_mode_ = true;
  void LateralMode(ControlFSMData<float>& data);
  float interleave_y[4] = {-0.08, 0.08, -0.02, 0.02};
  float UpdateBodyHeight(ControlFSMData<float> &data, float bodyheight);
  void UpdateInterLeaveY(ControlFSMData<float> &data, float* interleave_y);
  void InitRPYMode();
  void ChangeRPYMode();
  Vec3<float> UpdatepRobotFrame(ControlFSMData<float>& data, Vec3<float> offset, int leg);
  /// Add End

  /// Add Begin by wuchunming, 20210712, collision detection
  int contact_table_[4];
  int pressure_[4];

  void ContactDetection(ControlFSMData<float>& data, bool openContactDetection);
  /// Add End
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
