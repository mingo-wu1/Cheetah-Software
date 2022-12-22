#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH
///Add Begin by peibo,zhaoyudong,2021-07-20,optimize the calculation time of MPC algorithm.
extern const int g_mpc_horizon_length;
///Add End

////////////////////
// Controller
////////////////////
/*
（1）原理
参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
一个步态周期由horizonLength(10)个mpc周期组成
步态按1KHz处理 mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms
则步态周期为 10*30 =300ms
*/
ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, BZL_UserParameters* parameters) :
  _logger("ConvexMPCLocomotion"),
  iters_between_mpc_(_iterations_between_mpc),                                                                            //MPC的迭代次数，控制频率用
  dt_(_dt)                                                                                                                //控制周期
  /// Add End
  /* origin code
  trotting_(horizon_length_, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),                                          //以下的格式都一样，是一个默认设置。第一个向量时四条腿的的偏移量（相位差），第二个向量是mpc支撑腿的中一步持续时长（支撑时长）
  bounding_(horizon_length_, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding_(horizon_length_, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking_(horizon_length_, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping_(horizon_length_, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping_(horizon_length_, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping_(horizon_length_, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping_(horizon_length_, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing_(horizon_length_, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trot_running_(horizon_length_, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trot_running_(horizon_length_, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking_(horizon_length_, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2_(horizon_length_, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing_(horizon_length_, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random_(horizon_length_, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2_(horizon_length_, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
  */
{
  SetParams(parameters);
}

//3.初始化MPC
void ConvexMPCLocomotion::Initialize(){
  for(int leg = 0; leg < 4; leg++) 
    first_swing_[leg] = true;

  first_run_ = true;
  /// Add Begin by peibo zhaoyudong,2021-04-27,　it solves the problem that 
  //when pressing B button when fast backward in the sport mode, the fast backward appears when entering the sport mode again
  x_vel_des_ = 0;
  y_vel_des_ = 0;
  /// Add End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  startTransition = false;
  /// Add End 

  /// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
  InitRPYMode();
  /// Add End
}

void ConvexMPCLocomotion::SetParams(BZL_UserParameters* parameters){
  horizon_length_ = 10;                                                                                               //MPC的预测未来状态数量，预测未来的10个时间步，值越大MPC计算量越大，越大和越小都不行，电机40KHZ不支持
  iters_between_gait_seg_ = 27/2;
  gait_seg_ = 10;
  foot_height_ = .06;
  /// Add Begin by wuchunming, 2021-03-10, add BZL parameters
  raibert_coeff_ = .03;
  mu_ = 0.4;
  rforce_max_ = 120;
  alpha_ = 4e-5;
  params_ = parameters;                                                                                               //（1）定义用户参数，根据传进来的形参
  dt_mpc_ = dt_ * iters_between_mpc_;                                                                                 //（2）定义mpc运算周期，公式：mpc运算周期=迭代次数*控制周期   
  default_iters_between_mpc_ = iters_between_mpc_;                                                                    //（3）定义MPC的迭代次数
  QUADRUPED_INFO(_logger, "dt_: %.3f iterations: %d, dt_mpc_: %.3f", dt_, iters_between_mpc_, dt_mpc_);
  
  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameters
  setup_problem(dt_mpc_, horizon_length_, mu_, rforce_max_);                                                          //（4）二次规划器QP的参数配置，配置1）mpc运算周期、2）MPC的预测未来状态数量（分段数）、3）摩擦系数、4）最大力
  /// Mod End
  
  //setup_problem(dt_mpc_, horizon_length_, 0.4, 650); // DH
  rpy_compensate_[0] = 0;                                                                                             //（5）初始化变量欧拉角数据为0
  rpy_compensate_[1] = 0;
  rpy_compensate_[2] = 0;
  rpy_integral_[0] = 0;
  rpy_integral_[1] = 0;
  rpy_integral_[2] = 0;

  for(int i = 0; i < 4; i++)                                                                                          //（6）初始化四条腿的状态初次都是摆动状态
    first_swing_[i] = true;

  InitSparseMPC();                                                                                                    //（7）初始化稀疏MPC

  pos_body_des.setZero();                                                                                             //（8）初始化期望的位置、速度、加速度为0
  vel_body_des.setZero();
  acc_body_des.setZero();
   
  /// Add Begin by zhaoyudong, lihao, 2021-03-03, add BZL parameters, mod gait segment, mpc algorithm
  iters_between_mpc_ = iters_between_mpc_ * params_->factor_of_iterations_between_mpc;
  horizon_length_ = params_->horizon_length;
  iters_between_gait_seg_ = params_->iterations_between_gait_seg;
  gait_seg_ = params_->gait_seg;
  foot_height_ = params_->foot_height;
  dt_gait_seg_ = dt_ * iters_between_gait_seg_;
  raibert_coeff_ = params_->raibert_coeff;
  mu_ = params_->mu;
  rforce_max_ = parameters->fmax;
  alpha_ = parameters->alpha;
  /// Add End
  
  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  startTransition = false;
  /// Add End

  /// Mod Begin by zhaoyudong,lihao 2020-03-03, mod gait segment, mpc algorithm
  trotting_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,5,5,0), Vec4<int>(6,6,6,6),"Trotting");
  /// Mod Begin by zhaoyudong,peibo, 2021-05-13, modifying bounding_ gait
  bounding_ = OffsetDurationGait(gait_seg_, Vec4<int>(5,5,0,0),Vec4<int>(6,6,6,6),"Bounding");
  /// Ori Code:
  // bounding_(gait_seg_, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  /// Mod End
  //bounding_(gait_seg_, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,0,0,0),Vec4<int>(6,6,6,6),"Pronking");
  jumping_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping");
  //galloping_(gait_seg_, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping");
  //galloping_(gait_seg_, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping");
  galloping_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping");
  standing_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing");
  //trot_running_(gait_seg_, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running");
  trot_running_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running");
  walking_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking");
  walking2_ = OffsetDurationGait(gait_seg_, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2");
  pacing_ = OffsetDurationGait(gait_seg_, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing");
  /// Add Begin by  zhaoyudong peibo, 2021-02-25, add triangle gait
	triangle1243_ = OffsetDurationGait(12, Vec4<int>(3, 6, 0, 9), Vec4<int>(9, 9, 9, 9), "triangle");
	triangle1234_ = OffsetDurationGait(12, Vec4<int>(3, 6, 9, 0), Vec4<int>(9, 9, 9, 9), "triangle");
	triangle1423_ = OffsetDurationGait(12, Vec4<int>(3, 9, 0, 6), Vec4<int>(9, 9, 9, 9), "triangle");
	triangle1324_ = OffsetDurationGait(12, Vec4<int>(3, 9, 6, 0), Vec4<int>(9, 9, 9, 9), "triangle");
	/// Add End
  trotting_stair_ = OffsetDurationGait(gait_seg_, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7), "TrottingStair");
  random_ = MixedFrequncyGait(gait_seg_, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot");
  random2_ = MixedFrequncyGait(gait_seg_, Vec4<int>(8,16,16,8), 0.5, "Double Trot");
  /// Mod End
}

//4.重新计算 MPC时间
void ConvexMPCLocomotion::RecomputeTiming(int iterations_per_mpc) {
  iters_between_mpc_ = iterations_per_mpc;
  dt_mpc_ = dt_ * iterations_per_mpc;
}

void ConvexMPCLocomotion::SetupCommand(ControlFSMData<float> & data){
  /*（1）根据机型设置默认机体期望高度*/
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){ // 如果是MINI_CHEETAH机器人
    body_height_ = 0.29;
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    body_height_ = 0.45;
  }else{
    assert(false);
  }

  /*（2）使用或者不使用遥控手柄来设置XY\偏航的速度命令、步态命令、机身高度*/
  float x_vel_cmd, y_vel_cmd; // x，y两个方向的速度指令
  float filter(0.1); // 定义一个过滤器权重变量，值为0.1， 这个值决定期望状态的变化幅度
  if(data.controlParameters->use_rc){                                         // 采用手柄控制
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand; // 1设置获得遥控指令
    data.userParameters->cmpc_gait = rc_cmd->variable[0];                     // 2设置获得步态类型指令
    yaw_turn_rate_ = -rc_cmd->omega_des[2];                                   // 3设置机身偏航方向速度
    x_vel_cmd = rc_cmd->v_des[0];                                             // 4设置机身X方向速度指令
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;                                       // 5设置机身Y方向速度指令
    body_height_ += rc_cmd->height_variation * 0.08;                          // 6设置调整机身的高度
  }else{                                                                      // 如果不采用遥控手柄控制
    yaw_turn_rate_ = data._desiredStateCommand->rightAnalogStick[0];          // 设置机身偏航方向速度

    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];                // 设置机身X方向速度指令
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];                // 设置机身Y方向速度指令
  }

  /*（3）通过上一次的期望数据和本次遥控手柄输入的期望数据，计算机体的期望位姿【即XY的期望位置、期望偏航角度】*/
  /*（1）计算本次机体XY期望速度*/
  /// Mod Begin by peibo zhaoyudong, 2021-04-09,Velocity expected smoothing
  bool use_smooth = data.userParameters->liner_a_limit_enable > 0.1;          // 速度平滑
  float speed_step = (float)(fabs(data.userParameters->liner_a_max) * dt_);
  x_vel_des_ = velocityExpectedSmoothing(use_smooth,speed_step,filter,x_vel_cmd,x_vel_des_);
  y_vel_des_ = velocityExpectedSmoothing(use_smooth,speed_step,filter,y_vel_cmd,y_vel_des_);
  /// Ori Code :
  // x_vel_des_ = x_vel_des_ * (1 - filter) + x_vel_cmd * filter;             // 设置XY方向期望速度，注意：这里的计算经过权重滤波的
  // y_vel_des_ = y_vel_des_ * (1 - filter) + y_vel_cmd * filter;             // 公式：本次X方向期望速度=上一次X方向期望速度*（1-过滤器的权重变量）+ 遥控器机身X方向速度命令*过滤器的权重变量
  /// Mod End

  /*（2）计算本次机体期望PRY方向*/
  /// Add Begin by yuzhiyou, 2021-06-03, fix yaw_des_ bug for move
  if (first_run_)
    yaw_des_ = data._stateEstimator->getResult().rpy[2] + dt_ * yaw_turn_rate_; //计算本次期望偏航角度
  else
    yaw_des_ =  yaw_des_ + dt_ * yaw_turn_rate_;
  /// origin code
 // yaw_des_ = data._stateEstimator->getResult().rpy[2] + dt_ * yaw_turn_rate_;
  /// Add End

  roll_des_ = 0.; // 默认横滚动角=0
  pitch_des_ = 0.; // 默认俯仰角=0

}

template<>
void ConvexMPCLocomotion::Run(ControlFSMData<float>& data) {

    /// Add Begin by wuchunming, 2021-03-10, add BZL parameters
    iters_between_mpc_ = iters_between_mpc_ * params_->factor_of_iterations_between_mpc;
    dt_mpc_ = dt_ * iters_between_mpc_;
    horizon_length_ = g_mpc_horizon_length;//params_->horizon_length;
    //  iters_between_gait_seg_ = params_->iterations_between_gait_seg;
    gait_seg_ = params_->gait_seg;
    //  foot_height_ = params_->foot_height;
    //  dt_gait_seg_ = dt_ * iters_between_gait_seg_;
    raibert_coeff_ = params_->raibert_coeff;
    mu_ = params_->mu;
    rforce_max_ = params_->fmax;
    alpha_ = params_->alpha;
    /// Add End
    /// Add Begin by peibo 2021-04-29,0430 integration of related functions, adding stair operation
  if (data.userParameters->wbc_param_mode == STAIR)
  {
    iters_between_gait_seg_ = data.userParameters->iterationsBetweenGaitSeg_stair;
    dt_gait_seg_ = dt_ * iters_between_gait_seg_;
    foot_traj_plan_type_ = FootTrajectoryPlanType::CUBICSPLINE;
    /// Mod Begin by peibo, 2021-05-13, add the parameter of leg raising height in stair mode
    foot_height_ = data.userParameters->foot_height_stair;
    /// Ori Code:
    // foot_height_ = 0.15;
    /// Mod End
	}
  else
	{
		iters_between_gait_seg_ = data.userParameters->iterations_between_gait_seg;
		dt_gait_seg_ = dt_ * iters_between_gait_seg_;
		foot_traj_plan_type_ = FootTrajectoryPlanType::PARABOLA;
		foot_height_ = params_->foot_height;
	}
	/// Add End
  bool omniMode = false;

  // Command Setup
  SetupCommand(data); // 设置躯干高度，躯干位置和姿态

/// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
  body_height_ = UpdateBodyHeight(data, body_height_);
  /// Add End

  /// Mod Begin by peibo 2021-04-29,adding stair operation,add gamepad to control gait switching
	static double gaitNumber_old = -1;
	gait_number_ = data.userParameters->wbc_param_mode == STAIR ? data.userParameters->cmpc_gait_stair : data._desiredStateCommand->rcCommand->variable[0];//data.userParameters->cmpc_gait;
	if (fabs(gaitNumber_old - gait_number_) > 0.0001)
	{
		gaitNumber_old = gait_number_;
		QUADRUPED_INFO(_logger, "gait_number_ : %d", gait_number_);
	}
	/// Ori Code:
	//gait_number_ = data.userParameters->cmpc_gait;
	/// Mod End

  /// Mod Begin by  zhaoyudong peibo,  2021-02-25, add triangle gait
  if (gait_number_ >= 16){
    gait_number_ -= 16;
    omniMode = true;
  }
  /// Ori Code:
  // if (gait_number_ >= 10){
  //   gait_number_ -= 10;
  //   omniMode = true;
  // }
  /// Mod End
  
  /// Add Begin by peibo, 2021-04-09,Preprocessing speed and gait
  gait_number_ = speed_Gait_Processing(data,gait_number_);
  /// Add End 
  /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
  kpkd_parameter_adjustment_Processing(data,gait_number_);
  /// Add End
  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing_
  if(((gait_number_ == 4) && current_gait_ != 4) || first_run_)
  {
    stand_traj_[0] = seResult.position[0];
    stand_traj_[1] = seResult.position[1];
    stand_traj_[2] = 0.21;
    stand_traj_[3] = 0;
    stand_traj_[4] = 0;
    stand_traj_[5] = seResult.rpy[2];
    world_pos_des_[0] = stand_traj_[0];
    world_pos_des_[1] = stand_traj_[1];
  }

  // pick gait
  Gait* gait = &trotting_;

  if(gait_number_ == 1)
    gait = &bounding_;
  else if(gait_number_ == 2)
    gait = &pronking_;
  else if(gait_number_ == 3)
    gait = &random_;
  else if(gait_number_ == 4)
    gait = &standing_;
  else if(gait_number_ == 5)
    gait = &trot_running_;
  else if(gait_number_ == 6)
    gait = &random2_;
  else if(gait_number_ == 7)
    gait = &random2_;
  else if(gait_number_ == 8)
    gait = &pacing_;
  /// Add Begin by  zhaoyudong peibo  2021-02-25, add triangle gait
	else if (gait_number_ == 10)
		gait = &triangle1234_;
	else if (gait_number_ == 11)
		gait = &triangle1243_;
	else if (gait_number_ == 12)
		gait = &triangle1423_;
	else if (gait_number_ == 13)
		gait = &triangle1324_;
	else if (gait_number_ == 14)
	  gait = &trotting_stair_;
	/// Add End
  current_gait_ = gait_number_;

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  gait->setTransition(startTransition);
  /// Add End

  /// Mod Begin by zhaoyudong, lihao, 2020-03-04, mod gait segment, mpc algorithm
  gait->setIterations(iters_between_gait_seg_, iter_counter_);
  /// origin code
  //gait->setIterations(iters_between_mpc_, iter_counter_);
  /// Mod End

  jumping_.setIterations(iters_between_mpc_, iter_counter_);


  jumping_.setIterations(27/2, iter_counter_);

  //printf("[%d] [%d]\n", jumping_.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state_.trigger_pressed(jump_state_.should_jump(jumping_.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state_.should_jump(jumping_.getCurrentGaitPhase())) {
    gait = &jumping_;
    RecomputeTiming(27/2);
    body_height_ = body_height_jumping_;
    currently_jumping_ = true;

  } else {
    RecomputeTiming(iters_between_mpc_);
    currently_jumping_ = false;
  }

  if(body_height_ < 0.02) {
    body_height_ = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(x_vel_des_, y_vel_des_, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_integral_[1] += dt_*(pitch_des_ - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_integral_[0] += dt_*(roll_des_ - seResult.rpy[0])/v_robot[1];
  }

  rpy_integral_[0] = fminf(fmaxf(rpy_integral_[0], -.25), .25);
  rpy_integral_[1] = fminf(fmaxf(rpy_integral_[1], -.25), .25);
	/// Mod Begin by peibo 2021-04-29,adding stair operation
	if (data.userParameters->wbc_param_mode == STAIR)
	{
		rpy_compensate_[1] = updateBodyPitch(data) ;
	}
  else
	{
		rpy_compensate_[1] = params_->use_body_pitch_compute > 1e-6 ? updateBodyPitch(data) : 0;
	}
	/// Ori Code:
		/// Mod Begin by yuzhiyou, wuchunming, 2021-04-06, add updateBodyPitch for Calculate the inclination of the body to the ground
		//rpy_compensate_[1] = params_->use_body_pitch_compute > 1e-6 ? updateBodyPitch(data) : v_robot[0] * rpy_integral_[1];
		/// Ori Code:
		//rpy_compensate_[1] = v_robot[0] * rpy_integral_[1];
		/// Mod End
	/// Mod End

  rpy_compensate_[0] = 0;//v_robot[1] * rpy_integral_[0] * (gait_number_!=8);  //turn off for pronking_


  for(int i = 0; i < 4; i++) {
    pos_foot_world_[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing_) {
    /// Mod Begin by zhaoyudong,peibo,wangjie,2021-05-28,optimize anti kick function
    if (fabs(y_vel_des_ - seResult.vBody[1]) > 0.2)
    {
      world_pos_des_ = seResult.position + dt_ * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    }
    else
    {
      world_pos_des_ += dt_ * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    }
    /// Ori Code :
    //world_pos_des_ += dt_ * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    /// Mod End
    
  }

  // some first time initialization
  if(first_run_)
  {
    world_pos_des_[0] = seResult.position[0];
    world_pos_des_[1] = seResult.position[1];
    world_pos_des_[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      foot_swing_traj_[i].setHeight(0.05);
      foot_swing_traj_[i].setInitialPosition(pos_foot_world_[i]);
      foot_swing_traj_[i].setFinalPosition(pos_foot_world_[i]);

    }
    first_run_ = false;
  }

  // foot placement
  /// Mod Begin by zhaoyudong, lihao, 2020-03-03, mod gait segment, mpc algorithm
  for(int l = 0; l < 4; l++)
    swing_times_[l] = gait->getCurrentSwingTime(dt_gait_seg_, l);
  /// origin code
  //for(int l = 0; l < 4; l++)
  //  swing_times_[l] = gait->getCurrentSwingTime(dt_mpc_, l);
  /// Mod End

  float side_sign[4] = {-1, 1, -1, 1};
  
  /// Del Begin by wuchunming, 2021-03-09, {-0.08, 0.08, 0.02, -0.02}->{-0.08, 0.08, -0.02, 0.02}
  // float interleave_y[4] = {-0.08, 0.08, -0.02, 0.02};
  /// Del End

/// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
  UpdateInterLeaveY(data, interleave_y);
/// Add End

  /// origin code
  //float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  /// Mod End

  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {

    if(first_swing_[i]) {
      swing_time_remaining_[i] = swing_times_[i];
      /// Add Begin by yuzhiyou, peibo, 2021-04-23, 
			pos_zfoot_stance_[i] = pos_foot_world_[i][2];
			pos_xfoot_stance_[i] = pos_foot_world_[i][0];
			/// Add End
    } else {
      swing_time_remaining_[i] -= dt_;
    }
    //if(first_swing_[i]) {
    //foot_swing_traj_[i].setHeight(.05);
    foot_swing_traj_[i].setHeight(foot_height_);

    /// Mod Begin by wuchunming, 2021-03-09, mod .065->_abadLinkLength+_kneeLinkY_offset
    Vec3<float> offset(0, side_sign[i] * (data._quadruped->_abadLinkLength+data._quadruped->_kneeLinkY_offset), 0);
    /// origin code
    //Vec3<float> offset(0, side_sign[i] * .065, 0);
    /// Mod End
     
    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    /// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
    pRobotFrame = rpy_mode_ == RPYMODE_::YAWROLL ? UpdatepRobotFrame(data, offset, i): pRobotFrame;    
    v_abs = RPYMODE_::YAWROLL == rpy_mode_ ? 1 : v_abs;
    /// Add End

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

    /// Mod Begin by zhaoyudong,lihao,  2020-03-03, mod gait segment, mpc algorithm
    float stance_time = gait->getCurrentStanceTime(dt_gait_seg_, i);
    /// origin code
    //float stance_time = gait->getCurrentStanceTime(dt_mpc_, i);
    /// Mod End

    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -yaw_turn_rate_* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = x_vel_des_;
    des_vel[1] = y_vel_des_;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swing_time_remaining_[i]);

    //+ seResult.vWorld * swing_time_remaining_[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    
    /// Mod Begin by peibo,zhaoyudong, 2021-04-29, 0430 integration of related functions, adding stair operation
		float pfx_rel, pfy_rel;
		if (data.userParameters->wbc_param_mode == STAIR)
		{
			pfx_rel = seResult.vWorld[0] * (.5 + params_->cmpc_bonus_swing) * swing_times_[i] * dt_mpc_ +
				raibert_coeff_ * (seResult.vWorld[0] - v_des_world[0]) +
				(0.5f*seResult.position[2] / 9.81f) * (seResult.vWorld[1] * yaw_turn_rate_);
			pfy_rel = v_des_world[1] * .5 *  swing_times_[i] * dt_mpc_ +
				raibert_coeff_ * (seResult.vWorld[1] - v_des_world[1]) +
				(0.5f*seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * yaw_turn_rate_);
		}
		else
		{
      /// Mod Begin by peibo,wangjie, 2021-05-28,optimize anti kick function
      pfx_rel = seResult.vWorld[0] * (.5 + params_->cmpc_bonus_swing) * stance_time +
        raibert_coeff_ * (seResult.vWorld[0] - v_des_world[0]) +
        (0.5f*seResult.position[2] / 9.81f) * (seResult.vWorld[1] * yaw_turn_rate_);
      pfy_rel = seResult.vWorld[1] * .5 *  stance_time +
        raibert_coeff_ * (seResult.vWorld[1] - v_des_world[1]) +
        (0.5f*seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * yaw_turn_rate_);
     /// Ori Code:
     // pfx_rel = seResult.vWorld[0] * (.5 + params_->cmpc_bonus_swing) * stance_time +
     //   raibert_coeff_ * (seResult.vWorld[0] - v_des_world[0]) +
     //   (0.5f*seResult.position[2] / 9.81f) * (seResult.vWorld[1] * yaw_turn_rate_);
     // pfy_rel = v_des_world[1] * .5 *  stance_time +
     //   raibert_coeff_ * (seResult.vWorld[1] - v_des_world[1]) +
     //   (0.5f*seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * yaw_turn_rate_);
     /// Mod End
		}
    /// Ori Code:
      /// Mod Begin by zhaoyudong, lihao, 2020-03-03, BZL parameters,
      // float pfx_rel = seResult.vWorld[0] * (.5 + params_->cmpc_bonus_swing) * stance_time +
      //   raibert_coeff_*(seResult.vWorld[0]-v_des_world[0]) +
      //   (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*yaw_turn_rate_);
      /// origin code
      //float pfx_rel = seResult.vWorld[0] * (.5 + params_->cmpc_bonus_swing) * stance_time +
      //  .03f*(seResult.vWorld[0]-v_des_world[0]) +
      //  (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*yaw_turn_rate_);
      /// Mod End

      /// Mod Begin by zhaoyudong, lihao, 2020-03-03, BZL parameters, mod gait segment, mpc algorithm
      // float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dt_mpc_ +
      //   raibert_coeff_*(seResult.vWorld[1]-v_des_world[1]) +
      //   (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*yaw_turn_rate_);
      /// origin code
      //float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dt_mpc_ +
      //  .03f*(seResult.vWorld[1]-v_des_world[1]) +
      //  (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*yaw_turn_rate_);
      /// Mod End
    /// Mod End

    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    /// Add Begin by peibo,wangjie, 2021-05-28,optimize anti kick function
    antiKickOptimize(i,Pf[0],Pf[1],seResult);
    /// Add End

    /// Mod Begin by peibo,zhaoyudong, 2021-04-29, 0430 integration of related functions, adding stair operation
    Pf[2] = data.userParameters->wbc_param_mode == STAIR ? pos_zfoot_stance_[i] - 0.003 : Pf[2] = -0.003;
    /// Ori Code:
    // Pf[2] = -0.003;
    /// Mod End
    //Pf[2] = 0.0;
    foot_swing_traj_[i].setFinalPosition(Pf);

  }

  // calc gait
  iter_counter_++;

  // load LCM leg swing gains
  Kp_ << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance_ = 0*Kp_;


  Kd_ << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance_ = Kd_;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  
  /// Mod Begin by zhaoyudong, lihao, 2020-03-03, mod gait segment, mpc algorithm
  int* mpcTable = nullptr;
  mpcTable = gait->getMpcTable(horizon_length_, iter_counter_, iters_between_mpc_, iters_between_gait_seg_);
  /// origin code
  //int* mpcTable = gait->getMpcTable();
  /// Mod End

  /// Add Begin by wuchunming, 20210712, collision detection
  ContactDetection(data, false);
  /// Add End

  UpdateMPCIfNeeded(mpcTable, data, omniMode); // 功能：定时迭代更新计算MPC，并解算足端反作用力（重点）

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll_[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll_[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll_[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif
// 【C控制范畴】依次遍历四条腿进行解算，解算完成四条腿的控制指令一起发送给腿部控制器，保证控制是同步的
  // 注意这里的控制输入没有足端反作用力和关节KD参数，但是支撑相是有的，因为腿部控制器是对四条腿进行同时控制的（遍历的解算时候是单条腿解算，解算完成四条腿的控制指令一起发送给腿部控制器，保证控制是同步的）
  // 这里是否是接触状态在上面的步态gait就已经实现了，这里仅仅调用结果就可以
  for(int foot = 0; foot < 4; foot++)
  { /*（1）判断当前脚处于支撑状态还是摆动状态*/
    float contact_state = contactStates[foot];
    float swingState = swingStates[foot];
    // 原理:使用贝塞尔曲线对足端的轨迹进行规划，输出足端的位置轨迹和速度轨迹，然后把位置、速度、增益Kp/Kd作为控制输入，发送给WBC执行或者腿部控制器执行
    // 原理:注意这里的控制输入没有足端反作用力和关节KD参数，但是支撑相是有的，因为腿部控制器是对四条腿进行同时控制的（遍历的解算时候是单条腿解算，解算完成四条腿的控制指令一起发送给腿部控制器)
    if(swingState > 0) // foot is in swing
    {
      if(first_swing_[foot])                                                                                                  //（1）若刚从支撑相切换到摆动相
      {
        first_swing_[foot] = false;                                                                                           // 标志位置0
        foot_swing_traj_[foot].setInitialPosition(pos_foot_world_[foot]);                                                     // 初始化设置起始点为原点 
      }

#ifdef DRAW_DEBUG_SWINGS //与摆动调试有关系，运行的时候不会调用的
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {                                                                                       //贝塞尔曲线的轨迹，分100个点
          /// Add Begin by wuchunming, 2021-04-23, add select for selecting foot trajectory plan, select plan
          if(FootTrajectoryPlanType::PARABOLA == foot_traj_plan_type_){
            foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState + i * step, swing_times_[foot]);
          }else if(FootTrajectoryPlanType::CUBICSPLINE == foot_traj_plan_type_){
            foot_swing_traj_[foot].computeSwingTrajectoryBezierForStair(swingState + i * step, swing_times_[foot]);
          }else{}
          /// origin code
          // foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState + i * step, swing_times_[foot]);
          /// Add End
          debugPath->position[i] = foot_swing_traj_[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = foot_swing_traj_[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }

      /// Add Begin by wuchunming, 2021-04-23, add select for selecting foot trajectory plan, select plan
      if(FootTrajectoryPlanType::PARABOLA == foot_traj_plan_type_){
        foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState, swing_times_[foot]);                      //【重点！！！】计算贝塞尔曲线
      }else if(FootTrajectoryPlanType::CUBICSPLINE == foot_traj_plan_type_){
        foot_swing_traj_[foot].computeSwingTrajectoryBezierForStair(swingState, swing_times_[foot]);
      }else{}
      /// origin code
      // foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState, swing_times_[foot]);
      /// Add End

      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = foot_swing_traj_[foot].getPosition();
      actualSphere->position = pos_foot_world_[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      /// Add Begin by wuchunming, 2021-04-23, add select for selecting foot trajectory plan, select plan
      // if(data._desiredStateCommand->gamepadCommand->leftBumper){
      //   foot_traj_plan_type_ = FootTrajectoryPlanType::PARABOLA;
      // }else if(data._desiredStateCommand->gamepadCommand->back){
      //   foot_traj_plan_type_ = FootTrajectoryPlanType::CUBICSPLINE;
      // }else{}

      if(FootTrajectoryPlanType::PARABOLA == foot_traj_plan_type_){
        foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState, swing_times_[foot]);                         //（2）使用贝塞尔曲线，计算腿部足端轨迹,  //输入参数：当前在摆动相的位置，摆动相时间长度 ;//计算贝塞尔曲线
      }else if(FootTrajectoryPlanType::CUBICSPLINE == foot_traj_plan_type_){
        foot_swing_traj_[foot].computeSwingTrajectoryBezierForStair(swingState, swing_times_[foot]);
      }else{}

      /// origin code
      // foot_swing_traj_[foot].computeSwingTrajectoryBezier(swingState, swing_times_[foot]);
      /// Add End

      //      foot_swing_traj_[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);
      
      //（3）获得足端轨迹位置点和速度点
      Vec3<float> pDesFootWorld = foot_swing_traj_[foot].getPosition();                                           // 获得轨迹位置点
      Vec3<float> vDesFootWorld = foot_swing_traj_[foot].getVelocity();                                           // 获得轨迹速度点

      //（4）足端轨迹位置点和速度点先转换到机体坐标系下，然后再转到hip关节坐标系下
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position)                                  //先转换到机体坐标系等同于到机体坐标系下
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);                                   //再转换到机体坐标系等同于到髋关节坐标系下

      //（5）【默认使用wbc控制】为wbc控制更新参数，参数包括腿部位置、腿部速度、腿部加速度
      // Update for WBC
      pos_foot_world_des[foot] = pDesFootWorld;
      vel_foot_world_des[foot] = vDesFootWorld;
      acc_foot_world_des[foot] = foot_swing_traj_[foot].getAcceleration();
      
      //（6）判断是否使用WBC控制
      if(!data.userParameters->use_wbc){                                                                          //若不使用WBC,直接发送腿部控制命令到腿部控制器
        // Update leg control command regardless of the usage of WBIC
        //不管WBIC的使用情况如何，更新腿部控制命令都是这几个  //用腿部控制器控制腿部的运动
        data._legController->commands[foot].pDes = pDesLeg;                                                       //腿部期望位置
        data._legController->commands[foot].vDes = vDesLeg;                                                       //腿部期望速度
        data._legController->commands[foot].kpCartesian = Kp_;                                                    //腿部KP增益
        data._legController->commands[foot].kdCartesian = Kd_;                                                    //腿部KD增益
      }

    }

//原理
//使用贝塞尔曲线对足端的轨迹进行规划，输出足端的位置轨迹和速度轨迹，然后把位置、速度、增益Kp/Kd作为控制输入发送给WBC执行或者腿部控制器执行
//摆动腿使用贝塞尔曲线对足端的轨迹进行规划，输出足端的位置轨迹和速度轨迹
//支撑腿通过MPC计算出足端的反作用力
//然后把位置、速度、增益Kp/Kd、反作用力、关节Kd参数作为控制输入，发送给WBC执行或者腿部控制器执行
//注意支撑相的控制输入是有足端反作用力和关节KD参数的，因为腿部控制器是对四条腿进行同时控制的（遍历的解算时候是单条腿解算，解算完成四条腿的控制指令一起发送给腿部控制器)

    else // foot is in stance
    {
      first_swing_[foot] = true;                                                                              //设置刚从摆动相切换到支撑相状态

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pos_foot_world_[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      //（1）根据运动学坐标转换，通过摆动腿的位置和速度计算，足端的位置和速度
      Vec3<float> pDesFootWorld = foot_swing_traj_[foot].getPosition();                                       //获取摆动腿的位置
      Vec3<float> vDesFootWorld = foot_swing_traj_[foot].getVelocity();                                       //获取摆动腿的速度
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);                   //计算腿部足端位置
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);                                                             //计算腿部足端速度
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      //（2）若未使用WBC 发送数据到腿部控制器
      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;                                                   //四条腿腿部期望位置命令
        data._legController->commands[foot].vDes = vDesLeg;                                                   //四条腿腿部期望速度命令
          data._legController->commands[foot].kpCartesian = Kp_stance_;                                       //四条腿腿部KP参数命令
          data._legController->commands[foot].kdCartesian = Kd_stance_;                                       //四条腿腿部KD参数命令

        data._legController->commands[foot].forceFeedForward = f_fforce_[foot];                               //四条腿腿部反作用力命令
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;                          //关节KD参数命令

        //      foot_swing_traj_[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping                                                                           //（3）若使用WBC,直接发送数据到腿部控制器
        data._legController->commands[foot].pDes = pDesLeg;                                                   //四条腿腿部期望位置命令
        data._legController->commands[foot].vDes = vDesLeg;                                                   //四条腿腿部期望速度命令
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance_;                                      //四条腿腿部KP参数命令
        data._legController->commands[foot].kdCartesian = Kd_stance_;                                         //四条腿腿部KD参数命令
      }
      //            cout << "Foot " << foot << " force: " << f_fforce_[foot].transpose() << "\n";
      se_contactState[foot] = contact_state;                                                                  //（4）更新接触状态估计

      // Update for WBC                                                                                       //为WBC做好更新准备了
      //rforce_des[foot] = -f_fforce_[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pos_body_des[0] = world_pos_des_[0];
  pos_body_des[1] = world_pos_des_[1];
  pos_body_des[2] = body_height_;

  vel_body_des[0] = v_des_world[0];
  vel_body_des[1] = v_des_world[1];
  vel_body_des[2] = 0.;

  acc_body_des.setZero();

  pos_body_rpy_des[0] = 0.;
  /// Mod Begin by peibo , wuchunming 2021-06-03,adding stair operation, add gamepad stick to change rpy
  if ( STAIR == data.userParameters->wbc_param_mode || params_->use_body_pitch_compute > 1e-6)
  {
    pos_body_rpy_des[1] = updateBodyPitch(data);
  }
  else{
/// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
    LateralMode(data);
/// Add End
  }
	/// Ori Code:
		/// Mod Begin by yuzhiyou, wuchunming, 2021-04-06, add updateBodyPitch for Calculate the inclination of the body to the ground
		//pos_body_rpy_des[1] = params_->use_body_pitch_compute > 1e-6 ? updateBodyPitch(data) : 0.;
		/// Ori Code:
		//pos_body_rpy_des[1] = 0.;
		/// Mod End
	/// Mod End
  
  pos_body_rpy_des[2] = yaw_des_;

/// Add Begin by wuchunming, 20210702, add rpy mode function for locomotion
  vel_body_rpy_des[0] = 0.;
  vel_body_rpy_des[1] = 0.;
/// Add End

  vel_body_rpy_des[2] = yaw_turn_rate_;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update


}

template<>
void ConvexMPCLocomotion::Run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

// 【P预测范畴】功能：定时迭代更新计算MPC，并解算足端反作用力(重点)
void ConvexMPCLocomotion::UpdateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iters_between_mpc_ = 30; // 设置计算MPC的间隔时间
  if((iter_counter_ % iters_between_mpc_) == 0) // 当iterationCounter等于计算MPC的间隔时间，就进行一次MPC计算，用于控制频率
  {
    /*（1）先获得状态估计器的数据，并且计算世界坐标系下的机体速度*/
    auto seResult = data._stateEstimator->getResult();                                           //1）先获取状态估计器的数据，包括躯干的位置、方向、速度
    float* p = seResult.position.data();                                                         //2）定义并赋值在世界坐标系下的机体位置 

    Vec3<float> v_des_robot(x_vel_des_, y_vel_des_,0);                                           //3）定义并赋值身体坐标系下的机体速度
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot; //4）定义并赋值世界坐标系下的机体速度
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    /*（2）判断当前状态为站立状态还是非站立状态*/
    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral_);
    if(current_gait_ == 4) //若当前为站定状态时
    {
      //（1）初始化12个躯干轨迹参数
      float trajInitial[12] = { 
        roll_des_,                                                                        //1）期望横滚角
        pitch_des_ /*-hw_i->state_estimator->se_ground_pitch*/,                           //2）期望俯仰角
        (float)stand_traj_[5]/*+(float)stateCommand->data.stateDes[11]*/,                 //3）期望偏航角
        (float)stand_traj_[0]/*+(float)fsm->main_control_settings.p_des[0]*/,             //4）X位置 
        (float)stand_traj_[1]/*+(float)fsm->main_control_settings.p_des[1]*/,             //5）Y位置
        (float)body_height_/*fsm->main_control_settings.p_des[2]*/,                       //6）Z位置 
        0,0,0,0,0,0};                                                                     //7）其他参数设置为0

      //（2）变成mpc问题需要的格式 
      for(int i = 0; i < horizon_length_; i++)
        for(int j = 0; j < 12; j++)
          trajAll_[12*i+j] = trajInitial[j];                                              //把12个躯干轨迹参数整理成MPC问题的矩阵格式
    }

    else // 若当前为非站定状态时                                                                 
    {
      //(1)定义并赋值目标值：轨迹的X、Y参数，轨迹跟踪误差
      const float max_pos_error = .1;                                                     // 定义并赋值轨迹跟踪误差为0.1
      float xStart = world_pos_des_[0];                                                   // 定义并赋值轨迹的X参数设定为期望的X位置
      float yStart = world_pos_des_[1];                                                   // 定义并赋值轨迹的Y参数设定为期望的Y位置

      //（2）把目标值限制在误差范围内 //轨迹的X参数在误差范围外时，显示其在最大误差边界
      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;
      //轨迹的Y参数在误差范围外时，显示其在最大误差边界
      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;
      
      world_pos_des_[0] = xStart;
      world_pos_des_[1] = yStart;

      //（3）机体轨迹参数初始化      
      float trajInitial[12] = {(float)rpy_compensate_[0],  // 0
        (float)rpy_compensate_[1],    // 1
        yaw_des_,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)body_height_,      // 5
        0,                                        // 6
        0,                                        // 7
        yaw_turn_rate_,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      //（4）变成mpc问题需要的格式 //轨迹为当前时刻向后预测一个步态周期的轨迹 按匀速运动计算
      for(int i = 0; i < horizon_length_; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll_[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this  //从当前位置开始
        {
          //trajAll_[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll_[4] = hw_i->state_estimator->se_pBody[1];
          trajAll_[2] = seResult.rpy[2];
        }
        else
        {
          trajAll_[12*i + 3] = trajAll_[12 * (i - 1) + 3] + dt_mpc_ * v_des_world[0];
          trajAll_[12*i + 4] = trajAll_[12 * (i - 1) + 4] + dt_mpc_ * v_des_world[1];
          trajAll_[12*i + 2] = trajAll_[12 * (i - 1) + 2] + dt_mpc_ * yaw_turn_rate_;
        }
      }
    }
    Timer solveTimer;

    //（5）判断解算离散MPC还是稠密MPC，得到足端反作用力
    if(params_->cmpc_use_sparse > 0.5) {
      SolveSparseMPC(mpcTable, data);
    } else {  //一般都是计算稠密mpc，得到足端反作用力
      SolveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

// 解算稠密MPC，得到足端反作用力的方法（一般用这个）
void ConvexMPCLocomotion::SolveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult(); /*（1）先通过状态估计器获取世界坐标下状态估计值，赋值给机体对应的数据*/

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
  
  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  float Q[12] = {0};                                            //1）定义Q矩阵权重
  for(int i=0;i<3;++i)
    Q[i] = data.userParameters->Q1(i);
  for(int i=3;i<6;++i)
    Q[i] = data.userParameters->Q2(i-3);
  for(int i=6;i<9;++i)
    Q[i] = data.userParameters->Q3(i-6);
  for(int i=9;i<12;++i)
    Q[i] = data.userParameters->Q4(i-9);
  /// origin code
  //float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  /// Mod End

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
 
  /// Del Begin by wuchunming, 20210630
  // float yaw = seResult.rpy[2];                               // 2)偏航角
  /// Del End

  float* weights = Q;                                           // 3)权重Q矩阵

  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  float alpha = alpha_; // make setting eventually              // alpha
  /// origin code
  // float alpha = 4e-5                                         //4)alpha
  /// Mod End

  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();                          //5）机体位置
  float* v = seResult.vWorld.data();                            //6）机体速度
  float* w = seResult.omegaWorld.data();                        //7）机体角速度
  float* q = seResult.orientation.data();                       //8）机体方向

  /*（2）计算从质心指向足端的向量*/
  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pos_foot_world_[i%4][i/4] - seResult.position[i/4]; //质心指向足端的向量=足端坐标（x,y,z)减去质心的位置坐标（x,y,z)

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);
  /*(3)Alpha值限幅*/
  if(alpha > 1e-4) {
    QUADRUPED_INFO(_logger, "Alpha was set too high (%f) adjust to 1e-5", alpha);
    alpha = 1e-5;
  }
  /*（5）设置MPC的参数*/
  Vec3<float> pxy_act(p[0], p[1], 0);                                         //2）定义从IMU状态估计器中获取的机体XY位置的中间变量
  Vec3<float> pxy_des(world_pos_des_[0], world_pos_des_[1], 0);               //3）定义从世界坐标系中获取的期望机体XY位置的中间变量
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - body_height_;/*（4）计算质心高度上的偏差*/
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);                 //4）定义从IMU状态估计器中获取的机体XY速度的中间变量

  Timer t1;                                                                   //1）定义第一个计时器
  dt_mpc_ = dt_ * iters_between_mpc_;                                         /*5）设置计算MPC时间间隔 */                  
  
  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  setup_problem(dt_mpc_,horizon_length_,mu_,rforce_max_);                     /*6）二次规划器QP的参数配置*/
  /// origin code
  setup_problem(dt_mpc_,horizon_length_,0.4,120);
  /// Mod End
  
  //setup_problem(dt_mpc_,horizon_length_,0.4,650); //DH
  update_x_drag(x_comp_integral_);                                            /*7）z轴方向加速度受x轴方向速度的影响程度*/
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {                                         /*8）机体XY速度过大/过小处理*/
    //x_comp_integral_ += params_->cmpc_x_drag * pxy_err[0] * dt_mpc_ / vxy[0];
    x_comp_integral_ += params_->cmpc_x_drag * pz_err * dt_mpc_ / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral_);

  update_solver_settings(params_->jcqp_max_iter, params_->jcqp_rho,           /*9）更新qp求解器设置*/
      params_->jcqp_sigma, params_->jcqp_alpha, params_->jcqp_terminate, params_->use_jcqp);
  //t1.stopPrint("Setup MPC");

  /*（6）解MPC的过程，得到足端反作用力*/  //trajAll放的是期望状态轨迹
  Timer t2; //定义第二个计时器，测量解一次MPC的时间需要多久
  //cout << "dt_mpc_: " << dt_mpc_ << "\n";

  /// Mod Begin yuzhiyou 2021-04-13,
  const float *rpy = seResult.rpy.data();
  update_problem_data_floats_yzy(p, v, q, w, r, rpy, weights, trajAll_, alpha, mpcTable); //【非常重要】更新QP问题参数和解，每次解算出的horizon个组力 只取第一组 convex_mpc 5节开头
  /// Ori Code:
  //update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll_, alpha, mpcTable);
  /// Mod End

  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  /*（7）将世界坐标下力转换到机体坐标下*/
  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);                                               //获取索引号的结果

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_fforce_[leg] = -seResult.rBody * f;                                                 //将世界坐标下力转换到机体坐标下
    // Update for WBC
    rforce_des[leg] = f;                                                                  //给WBC更新力
  }
}

// 解算sparseMPC，得到足端反作用力
void ConvexMPCLocomotion::SolveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizon_length_; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizon_length_; i++) {
    for(u32 j = 0; j < 12; j++) {
      sparse_traj_[i][j] = trajAll_[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pos_foot_world_[foot][axis] - seResult.position[axis];
    }
  }

  sparse_cmpc_.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  sparse_cmpc_.setContactTrajectory(contactStates.data(), contactStates.size());
  sparse_cmpc_.setStateTrajectory(sparse_traj_);
  sparse_cmpc_.setFeet(feet);
  sparse_cmpc_.run();

  Vec12<float> resultForce = sparse_cmpc_.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_fforce_[foot] = -seResult.rBody * force;
    rforce_des[foot] = force;
  }
}

// 初始化稀疏MPC
void ConvexMPCLocomotion::InitSparseMPC() {
  Mat3<double> baseInertia; // 定义机体惯性
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9; // 定义质量

  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  double maxForce = rforce_max_; // 定义最大扭矩
  /// origin code
  //double maxForce = 120;
  /// Mod End

  std::vector<double> dtTraj;
  for(int i = 0; i < horizon_length_; i++) {
    dtTraj.push_back(dt_mpc_);
  }

  Vec12<double> weights; // 定义权重

  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  //for(int i=0;i<3;++i)
  //  weights << params_->weights1[i];
  //for(int i=3;i<6;++i)
  //  weights << params_->weights2[i-3];
  //for(int i=6;i<9;++i)
  //  weights << params_->weights3[i-6];
  //for(int i=9;i<12;++i)
  //  weights << params_->weights4[i-9];
  /// origin code
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  /// Mod End

  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  sparse_cmpc_.setRobotParameters(baseInertia, mass, maxForce); // 设置机器人参数

  /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
  sparse_cmpc_.setFriction(mu_); // 设置摩擦系数
  /// origin code
  //sparse_cmpc_.setFriction(0.4);
  /// Mod End

  sparse_cmpc_.setWeights(weights, 4e-5); // 设置权重
  sparse_cmpc_.setDtTrajectory(dtTraj); // 设置轨迹

  sparse_traj_.resize(horizon_length_);
}

/// Add Begin by yuzhiyou, wuchunming, 2021-04-06, add updateBodyPitch for Calculate the inclination of the body to the ground
/**
 * @brief Calculate the inclination of the body to the ground
 * @param [in] ControlFSMData, data of ControlFSM
 * @return bodyPitch, inclination of the body to the ground 
 */
float ConvexMPCLocomotion::updateBodyPitch(ControlFSMData<float>& data){
  Vec3<float> p_f(0, 0, 0);
  Vec3<float> p_h(0 ,0, 0);
  float bodyPitch(0);
  for (int i = 0; i < 4; i++){
    Quadruped<float> &quadruped =
            *(data._legController->datas->quadruped);
    Vec3<float> ph = quadruped.getHipLocation(i);
    Vec3<float> p_rel = ph + data._legController->datas[i].p;      
    float phase = fmin(data._stateEstimator->getResult().contactEstimate(i), float(1));
    if (phase > 0){
      if (i < 2)
        p_f=p_rel;
      else
        p_h=p_rel;
    }
  }  
  bodyPitch = data._stateEstimator->getResult().rpy(1) - std::atan2(p_f(2)- p_h(2), p_f(0)-p_h(0));  
  return bodyPitch;
}
/// Add End 

/// Add Begin by peibo, 2021-04-09,Preprocessing speed and gait
/**
 * @brief Automatically modify PID parameters and gait as speed changes
 * @param [in] ControlFSMData, data of ControlFSM
 * @param [in] gait_number_, Current gait
 * @return gait_number_, Modified gait number 
 */
int ConvexMPCLocomotion::speed_Gait_Processing(ControlFSMData<float>& data,int gait_number_)
{
  /// Add Begin by peibo, 2021-03-29,automatically change kp,kd parameters with speed 
  double speed_value = sqrt(x_vel_des_*x_vel_des_ + y_vel_des_ * y_vel_des_);
  double des_speed_level = 0;
  if (speed_value >= data.userParameters->speed_threshold_value_2)
  {
	  des_speed_level = 2;
  }
  else if(speed_value >= data.userParameters->speed_threshold_value_1)
  {
	  des_speed_level = 1;
  }
  else
  {
	  des_speed_level = 0;
  }
  if (fabs(data.userParameters->cur_speed_level - des_speed_level) > 0.001)
  {
	  data.userParameters->cur_speed_level = des_speed_level;
	  QUADRUPED_INFO(_logger, "cur_speed_level changed : %f", data.userParameters->cur_speed_level);
  }
  /// Add End
  if (fabs(speed_value) >= data.userParameters->speed_threshold_value_1 && gait_number_ == 9)
  {
	  gait_number_ = 5;
  }
  /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
  kpkd_parameter_adjustment_Processing(data,gait_number_);
  /// Add End
  return gait_number_;
}
/// Add End 
/// Add Begin by peibo zhaoyudong, 2021-04-09,Speed expected smoothing
/**
 * @brief Speed expected smoothing
 * @param [in] use_smooth, Ｉs smoothing enabled
 * @param [in] speed_step, Maximum speed variation
 * @param [in] filter, filter
 * @param [in] vel_des, Speed expectation
 * @param [in] vel_source, Current speed value
 * @return Modified speed expectation 
 */
float ConvexMPCLocomotion::velocityExpectedSmoothing(bool use_smooth,float speed_step,float filter,float vel_des,float vel_source)
{
  double vel_filter = vel_source * (1 - filter) + vel_des * filter;
  if (use_smooth)
  {
	  if (fabs(vel_filter - vel_source) > speed_step)
	  {
		  return ((vel_filter - vel_source) / fabs(vel_filter - vel_source) * speed_step + vel_source);
	  }
  }
  return vel_filter;
}
/// Add End
/// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
void ConvexMPCLocomotion::kpkd_parameter_adjustment_Processing(ControlFSMData<float>& data,int gait_number_)
{
  if(data.userParameters->wbc_param_mode == STAIR)
  {
      data.userParameters->Kp_joint_fr_wbc = data.userParameters->Kp_joint_fr_stair;
      data.userParameters->Kd_joint_fr_wbc = data.userParameters->Kd_joint_fr_stair;

      data.userParameters->Kp_joint_fl_wbc = data.userParameters->Kp_joint_fl_stair;
      data.userParameters->Kd_joint_fl_wbc = data.userParameters->Kd_joint_fl_stair;

      data.userParameters->Kp_joint_br_wbc = data.userParameters->Kp_joint_br_stair;
      data.userParameters->Kd_joint_br_wbc = data.userParameters->Kd_joint_br_stair;

      data.userParameters->Kp_joint_bl_wbc = data.userParameters->Kp_joint_bl_stair;
      data.userParameters->Kd_joint_bl_wbc = data.userParameters->Kd_joint_bl_stair;
  }
  else if(gait_number_ == 9 || gait_number_ == 5)
  {
    switch ((int)data.userParameters->cur_speed_level)
    {
    case 1:
      data.userParameters->Kp_joint_fr_wbc = data.userParameters->Kp_joint_fr_heighSpeed_1;
      data.userParameters->Kd_joint_fr_wbc = data.userParameters->Kd_joint_fr_heighSpeed_1;

      data.userParameters->Kp_joint_fl_wbc = data.userParameters->Kp_joint_fl_heighSpeed_1;
      data.userParameters->Kd_joint_fl_wbc = data.userParameters->Kd_joint_fl_heighSpeed_1;

      data.userParameters->Kp_joint_br_wbc = data.userParameters->Kp_joint_br_heighSpeed_1;
      data.userParameters->Kd_joint_br_wbc = data.userParameters->Kd_joint_br_heighSpeed_1;

      data.userParameters->Kp_joint_bl_wbc = data.userParameters->Kp_joint_bl_heighSpeed_1;
      data.userParameters->Kd_joint_bl_wbc = data.userParameters->Kd_joint_bl_heighSpeed_1;
      break;
    case 2:
      data.userParameters->Kp_joint_fr_wbc = data.userParameters->Kp_joint_fr_heighSpeed_2;
      data.userParameters->Kd_joint_fr_wbc = data.userParameters->Kd_joint_fr_heighSpeed_2;

      data.userParameters->Kp_joint_fl_wbc = data.userParameters->Kp_joint_fl_heighSpeed_2;
      data.userParameters->Kd_joint_fl_wbc = data.userParameters->Kd_joint_fl_heighSpeed_2;

      data.userParameters->Kp_joint_br_wbc = data.userParameters->Kp_joint_br_heighSpeed_2;
      data.userParameters->Kd_joint_br_wbc = data.userParameters->Kd_joint_br_heighSpeed_2;

      data.userParameters->Kp_joint_bl_wbc = data.userParameters->Kp_joint_bl_heighSpeed_2;
      data.userParameters->Kd_joint_bl_wbc = data.userParameters->Kd_joint_bl_heighSpeed_2;
      break;
    case 0:
    default:
      data.userParameters->Kp_joint_fr_wbc = data.userParameters->Kp_joint_ex1;
      data.userParameters->Kd_joint_fr_wbc = data.userParameters->Kd_joint_ex1;

      data.userParameters->Kp_joint_fl_wbc = data.userParameters->Kp_joint_ex2;
      data.userParameters->Kd_joint_fl_wbc = data.userParameters->Kd_joint_ex2;

      data.userParameters->Kp_joint_br_wbc = data.userParameters->Kp_joint_ex3;
      data.userParameters->Kd_joint_br_wbc = data.userParameters->Kd_joint_ex3;

      data.userParameters->Kp_joint_bl_wbc = data.userParameters->Kp_joint_ex4;
      data.userParameters->Kd_joint_bl_wbc = data.userParameters->Kd_joint_ex4;
      break;
    }
  }
  else
  {
      data.userParameters->Kp_joint_fr_wbc = data.userParameters->Kp_joint_fr_bounding_gait;
      data.userParameters->Kd_joint_fr_wbc = data.userParameters->Kd_joint_fr_bounding_gait;

      data.userParameters->Kp_joint_fl_wbc = data.userParameters->Kp_joint_fl_bounding_gait;
      data.userParameters->Kd_joint_fl_wbc = data.userParameters->Kd_joint_fl_bounding_gait;

      data.userParameters->Kp_joint_br_wbc = data.userParameters->Kp_joint_br_bounding_gait;
      data.userParameters->Kd_joint_br_wbc = data.userParameters->Kd_joint_br_bounding_gait;

      data.userParameters->Kp_joint_bl_wbc = data.userParameters->Kp_joint_bl_bounding_gait;
      data.userParameters->Kd_joint_bl_wbc = data.userParameters->Kd_joint_bl_bounding_gait;
  }

}

/// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
bool ConvexMPCLocomotion::Transition()
{
  static int continueIndex = 0;
  if(startTransition == false)
    continueIndex = 0;
  startTransition = true;
  bool transitionComplete = true;
  for(int leg = 0; leg < 4; leg++)
  {
    if(fabs(contact_state[leg] - 0) < 1e-6)
    {
      transitionComplete = false;
      break;
    }
  }
  if(transitionComplete && ++continueIndex > 10)
  {
    return true;
  }
  return false;
}
/// Add End 

/// Add Begin by niuxinjian, wuchunming, 2021-03-30, Trajectory Plan
/**
 * @brief Cubic Interpolation Method for Trajectory Plan
 * @param [in] gamepad_des_theta, gamepad destination theta
 * @param [in] plan_deltaT, trajectory plan deltaT
 * @return plan_body_des_theta, trajectory plan destination theta of body 
 */
float ConvexMPCLocomotion::TrajectoryPlan(TrajectoryPlanData<float> &data, float _gamepad_des_theta, float _plan_deltaT){
  (void)_plan_deltaT;
  data.gamepad_des_theta = _gamepad_des_theta;

  if(data.gamepad_des_theta - data.gamepad_pre_theta > 0.000001 || data.gamepad_des_theta - data.gamepad_pre_theta < -0.000001 ){
      data.plan_time_step = 0.0;
      data.plan_deltaT = 2; /* std::fabs(data.gamepad_des_theta - data.plan_body_des_theta);*/
      data.plan_body_pre_theta = data.plan_body_des_theta;
  }else{}

  if(data.plan_time_step < data.plan_deltaT){
    data.plan_time_step += 0.002;
    data.plan_body_des_theta = data.plan_body_pre_theta + 0 * data.plan_time_step + 
            3 * (data.gamepad_des_theta - data.plan_body_pre_theta ) / ( data.plan_deltaT * data.plan_deltaT ) * data.plan_time_step * data.plan_time_step + 
            (-2) * (data.gamepad_des_theta - data.plan_body_pre_theta) / ( data.plan_deltaT * data.plan_deltaT * data.plan_deltaT ) * data.plan_time_step * data.plan_time_step * data.plan_time_step;
  }else{}

  data.gamepad_pre_theta = data.gamepad_des_theta;

  return data.plan_body_des_theta;
}

void ConvexMPCLocomotion::footPos(){

}

void ConvexMPCLocomotion::LateralMode(ControlFSMData<float>& data){
  (void)data;
  // if(x_vel_des_ <= 0.5 /*&& pos_body_rpy_des[0] >= 0.3*/){
  //   allow_lateral_mode_ = true;
  // }else{
  //   allow_lateral_mode_ = false;
  //   if(pos_body_rpy_des[0] > 0.3){
  //     // do something
  //   }
  // }

//data._desiredStateCommand->gamepadCommand->rightTriggerButton

  // if(true == allow_lateral_mode_)
  // {
    // pos_body_rpy_des[1] = 0.;
    auto gpCmd = data._desiredStateCommand->gamepadCommand;
    /// Add Begin by wuchunming, 2021-06-03, add right stick button 
    if(0 == gpCmd->rightTriggerButton && true == allow_rpy_mode_switch_)
    {
      ++rpy_mode_;
      rpy_mode_ = rpy_mode_ % RPYMODE_::count;
      allow_rpy_mode_switch_ = false;
    }
    else if(1 == gpCmd->rightTriggerButton){
      allow_rpy_mode_switch_ = true;
      ChangeRPYMode();
    }
    else{}
    
    if(RPYMODE_::YAW == rpy_mode_){
      //std::cout<<"YAW"<<std::endl;
      // InitLateralMode();
    }
    else if(RPYMODE_::YAWPITCH == rpy_mode_)
    {
      //std::cout<<"YAWPITCH"<<std::endl;
      // pos_body_rpy_des[1] = fabs(data._desiredStateCommand->gamepadCommand->rightStickAnalog[1]) < 0.6 ? 
      //                       0 : data._desiredStateCommand->gamepadCommand->rightStickAnalog[1];
      // if(pos_body_rpy_des[1] < 0)
      //   pos_body_rpy_des[1] = -(pos_body_rpy_des[1] + 0.5);
      // else if(pos_body_rpy_des[1] > 0)
      //   pos_body_rpy_des[1] = -(pos_body_rpy_des[1] - 0.5);
      // else{}
      constexpr float MAX_P_RAD = 0.7;
      pos_body_rpy_des[1] = fabs(gpCmd->rightStickAnalog[1]) < MAX_P_RAD ? 0 : gpCmd->rightStickAnalog[1];
      pos_body_rpy_des[1] = fabs(pos_body_rpy_des[1]) > 0 ? -pos_body_rpy_des[1] + MAX_P_RAD * fabs(pos_body_rpy_des[1]) / pos_body_rpy_des[1] : pos_body_rpy_des[1];

      // yaw_turn_rate_ = data._desiredStateCommand->rightAnalogStick[0];
      // data._stateEstimator->getResult().rpy[2] + dt_ * yaw_turn_rate_;
      // pitch_turn_rate_ = pos_body_rpy_des[1];
      // pos_body_rpy_des[1] = data._stateEstimator->getResult().rpy[1] + dt_ * pitch_turn_rate_;
      // vel_body_rpy_des[1] = pitch_turn_rate_;
      pos_body_rpy_des[1] = TrajectoryPlan(yp_data1_, pos_body_rpy_des[1], 1);
    }
    else if(RPYMODE_::ROLLPITCH == rpy_mode_)
    {
      //std::cout<<"ROLLPITCH"<<std::endl;
      // pos_body_rpy_des[0] = fabs(data._desiredStateCommand->gamepadCommand->rightStickAnalog[0]) < 0.85 ? 
      //                       0 : data._desiredStateCommand->gamepadCommand->rightStickAnalog[0];
      // pos_body_rpy_des[1] = fabs(data._desiredStateCommand->gamepadCommand->rightStickAnalog[1]) < 0.6 ?
      //                       0 : -data._desiredStateCommand->gamepadCommand->rightStickAnalog[1];
      // if(pos_body_rpy_des[0] < 0)
      //   pos_body_rpy_des[0] = pos_body_rpy_des[0] + 0.85;
      // else if(pos_body_rpy_des[0] > 0)
      //   pos_body_rpy_des[0] = pos_body_rpy_des[0] - 0.85;
      // else{}
      constexpr float MAX_RP_RAD = 0.7;
      // pos_body_rpy_des[0] += data._stateEstimator->getResult().rpy[0];
      // pos_body_rpy_des[1] += data._stateEstimator->getResult().rpy[1];      
      pos_body_rpy_des[0] = fabs(gpCmd->rightStickAnalog[0]) < MAX_RP_RAD ? 0 : gpCmd->rightStickAnalog[0];
      pos_body_rpy_des[1] = fabs(gpCmd->rightStickAnalog[1]) < MAX_RP_RAD ? 0 : -gpCmd->rightStickAnalog[1];
      pos_body_rpy_des[0] = fabs(pos_body_rpy_des[0]) > 0 ? pos_body_rpy_des[0] - MAX_RP_RAD * fabs(pos_body_rpy_des[0]) / pos_body_rpy_des[0] : pos_body_rpy_des[0];
      pos_body_rpy_des[1] = fabs(pos_body_rpy_des[1]) > 0 ? pos_body_rpy_des[1] - MAX_RP_RAD * fabs(pos_body_rpy_des[1]) / pos_body_rpy_des[1] : pos_body_rpy_des[1];

      // if(pos_body_rpy_des[1] < 0)
      //   pos_body_rpy_des[1] = pos_body_rpy_des[1] + 0.6;
      // else if(pos_body_rpy_des[1] > 0)
      //   pos_body_rpy_des[1] = pos_body_rpy_des[1] - 0.6;
      // else{}
      pos_body_rpy_des[0] = TrajectoryPlan(rp_data2_, pos_body_rpy_des[0], 1);
      pos_body_rpy_des[1] = TrajectoryPlan(rp_data3_, pos_body_rpy_des[1], 1);
    }
    else if(RPYMODE_::YAWROLL == rpy_mode_)
    {
      //std::cout<<"YAWROLL"<<std::endl;
      static float rollRadDes = 0.;
      constexpr float MAX_R_RAD = 1.5;
      float gpRollRadDes = gpCmd->rightStickAnalog[1];
      rollRadDes = gpRollRadDes;
      rollRadDes = fabs(gpRollRadDes) > 0.6 ? (rollRadDes + 0.001 * fabs(gpRollRadDes) / gpRollRadDes) : rollRadDes;
      if(0 == gpRollRadDes){
        pos_body_rpy_des[0] = data._stateEstimator->getResult().rpy[0];  
      }else if(gpRollRadDes > 0){
        pos_body_rpy_des[0] = MAX_R_RAD;
      }else if(gpRollRadDes < 0){
        pos_body_rpy_des[0] = 0;
      }
      pos_body_rpy_des[0] = TrajectoryPlan(yr_data4_, pos_body_rpy_des[0], 1);

      // rollRadDes = gpRollRadDes;
      // rollRadDes = fabs(gpRollRadDes) > 0.6 ? (rollRadDes + 0.001 * fabs(gpRollRadDes) / gpRollRadDes) : rollRadDes;
      // pos_body_rpy_des[0] = rollRadDes >= MAX_RAD ? MAX_RAD : rollRadDes <= 0 ? 0 : rollRadDes;
      // pos_body_rpy_des[0] = data._stateEstimator->getResult().rpy[0] + 0.5;
      // pos_body_rpy_des[0] = TrajectoryPlan(yr_data4_, pos_body_rpy_des[0], 1);
      // pos_body_rpy_des[0] = rollDes;
      // std::cout<<", r_des:"<<r_des<<", "<<std::endl;

      // pos_body_rpy_des[0] = fabs(data._desiredStateCommand->gamepadCommand->rightStickAnalog[1]) < 0.5 ?
      //                       0 : -data._desiredStateCommand->gamepadCommand->rightStickAnalog[1];
      // if(pos_body_rpy_des[0] < 0){
      //   pos_body_rpy_des[0] += 0.5;//[-0.5,0]
      // }
      // else if(pos_body_rpy_des[0] > 0){
      //   pos_body_rpy_des[0] -= 0.5;//[0,0.5]
      // }
      // else{}
      // pos_body_rpy_des[0] = params_->body_roll_rad;
      // pos_body_rpy_des[0] = TrajectoryPlan(data4, pos_body_rpy_des[0],1);
      // roll_turn_rate_ = pos_body_rpy_des[0];
      // pos_body_rpy_des[0] = data._stateEstimator->getResult().rpy[0] + dt_ * roll_turn_rate_;
      // vel_body_rpy_des[0] = roll_turn_rate_;
    // }
    // else{}
  }
  else{
    QUADRUPED_INFO(_logger, "count");
    //do something
  }
}
/// Add End 

float ConvexMPCLocomotion::UpdateBodyHeight(ControlFSMData<float> &data, float bodyheight){
  float H_MAX = -0.15;
  height_offset_ += 0.0005 * (bool)data._desiredStateCommand->gamepadCommand->buttonRight;
  height_offset_ -= 0.0005 * (bool)data._desiredStateCommand->gamepadCommand->buttonLeft;
  height_offset_ = height_offset_ >= H_MAX ? (height_offset_ <= 0 ? height_offset_ : 0) : H_MAX;
  bodyheight += height_offset_;
  return bodyheight;
}

void ConvexMPCLocomotion::UpdateInterLeaveY(ControlFSMData<float> &data, float* interleave_y){
  if(RPYMODE_::YAWROLL == rpy_mode_){
(void)data;
/*
      (void)interleave_y;
      data.userParameters->Q4[1] = 2;
      data.userParameters->Q4[1] = 0.1;
      
      float k = 0.1;
      float max_y = 0.1 * data._stateEstimator->getResult().rpy[0] * data._stateEstimator->getResult().rpy[0];
      //interleave_y[0] += +dt_ * k;
      //interleave_y[1] += -dt_ * k;
      interleave_y[2] += -dt_ * k;
      //interleave_y[3] += -dt_ * k;
      //interleave_y[0] = interleave_y[0] >= max_y * 0.3 ?  max_y * 0.3: interleave_y[0];//{+0.58, -0.58, +0.52, -0.52};
      //interleave_y[1] = interleave_y[1] <=-max_y ? -max_y : interleave_y[1];//{+0.58, -0.58, +0.52, -0.52};
      interleave_y[2] = interleave_y[2] >= max_y * 0.2 ?  max_y * 0.2 : interleave_y[2];//{+0.58, -0.58, +0.52, -0.52};
      //interleave_y[3] = interleave_y[3] <=-max_y * 1.5 ? -max_y * 1.5: interleave_y[3];//{+0.58, -0.58, +0.52, -0.52};
      //std::cout<<"max_y"<<max_y<<"interleave_y:"<<interleave_y[0]<<", "<<interleave_y[1]<<", "<<interleave_y[2]<<", "<<interleave_y[3]<<std::endl;
      // auto res = data._stateEstimator->getResult();
      // std::cout<<", r:"<<res.rpy[0]<<std::endl;
*/
  }
  else if(RPYMODE_::ROLLPITCH == rpy_mode_){
  interleave_y[0] = 0;
  interleave_y[1] = 0;
  interleave_y[2] = 0;
  interleave_y[3] = 0;
 
     
      //yaw_turn_rate_ = 0;
      //yaw_des_ = data._stateEstimator->getResult().rpy[2] + dt_ * yaw_turn_rate_;
  }
  else if(RPYMODE_::YAW == rpy_mode_){
      ChangeRPYMode();
  }
  else if(RPYMODE_::YAWPITCH == rpy_mode_){}
  else{}
}

void ConvexMPCLocomotion::InitRPYMode(){
  yp_data1_.setZero();
  rp_data2_.setZero();
  rp_data3_.setZero();
  yr_data4_.setZero();
  pos_body_rpy_des[0] = 0;
  pos_body_rpy_des[1] = 0;
  rpy_mode_ = RPYMODE_::YAW;
  body_height_ = 0.29;
  interleave_y[0] = -0.08;
  interleave_y[1] = 0.08;
  interleave_y[2] = -0.02;
  interleave_y[3] = 0.02;
}

void ConvexMPCLocomotion::ChangeRPYMode(){
  yp_data1_.setZero();
  rp_data2_.setZero();
  rp_data3_.setZero();
  yr_data4_.setZero();
  pos_body_rpy_des[0] = 0;
  pos_body_rpy_des[1] = 0;
  interleave_y[0] = -0.08;
  interleave_y[1] = 0.08;
  interleave_y[2] = -0.02;
  interleave_y[3] = 0.02;
}

Vec3<float> ConvexMPCLocomotion::UpdatepRobotFrame(ControlFSMData<float>& data, Vec3<float> offset, int leg){
    //auto Roll = data._stateEstimator->getResult().rpy[0];
    Mat3<float> pRollCorrected = coordinateRotation(CoordinateAxis::X, pos_body_rpy_des[0]);// gamepad
    // Mat3<float> pRollCorrected = coordinateRotation(CoordinateAxis::X, Roll); // estimator Roll
    Vec3<float> pRobotFrame = pRollCorrected * (data._quadruped->getHipLocation(leg) + offset);

    float theta0 = data._legController->datas[0].q[0];
    float theta1 = data._legController->datas[1].q[0];
    float theta2 = data._legController->datas[2].q[0];
    float theta3 = data._legController->datas[3].q[0];
    QUADRUPED_INFO(_logger, "r_des:%f, r:%f, theta0:%f, theta1:%f, theta2:%f, theta3:%f",
      pos_body_rpy_des[0], data._stateEstimator->getResult().rpy[0], theta0, theta1, theta2, theta3);

    return pRobotFrame;
}
/*
Vec3<float> ConvexMPCLocomotion::UpdatepRobotFrame(ControlFSMData<float>& data, Vec3<float> offset, int leg){
    Eigen::Matrix<float,3,3> RotateAbadOffset;
    float theta = data._legController->datas[leg].q[0];
    RotateAbadOffset << 1, 0              , 0               ,
                        0, std::cos(theta), -std::sin(theta),
                        0, std::sin(theta),  std::cos(theta);
    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(leg) + RotateAbadOffset * offset);
    //offset[2] = std::sin(theta) * offset[1];
    //offset[1] = std::cos(theta) * offset[1];
    return pRobotFrame;
}
*/

/// Add End 

/// Add Begin by wuchunming, 20210712, collision detection
void ConvexMPCLocomotion::ContactDetection(ControlFSMData<float>& data, bool openContactDetection){
  if(openContactDetection)
  {
    if(data.sensorData_)
       for(int leg = 0; leg < 4; ++leg)
         pressure_[leg] = (unsigned int)(unsigned short)data.sensorData_->GetData()[leg];
    QUADRUPED_INFO(_logger, "[%d %d %d %d]",
      pressure_[0], pressure_[1], pressure_[2], pressure_[3]);
    for(int leg = 0; leg < 4; ++leg){
      contact_table_[leg] = pressure_[leg] > 1000 ? 1 : 0;
    }
  }
}
/// Add End

/// Add Begin by peibo,wangjie, 2021-05-28,optimize anti kick function
void ConvexMPCLocomotion::antiKickOptimize(int legIndex,float& pfX,float& pfY,const StateEstimate<float>& seResult)
{
    float diff_y_vel_pre = fabs(y_vel_des_ - seResult.vBody[1]) - 0.2;
    diff_y_vel_pre = diff_y_vel_pre < 0 ? 0 : diff_y_vel_pre;
    float diff_y_vel = 4 * diff_y_vel_pre;
    float expDiff = exp(diff_y_vel);
    float expDiffInv = exp(-diff_y_vel);
    float finalVelParam = (expDiff - expDiffInv) / (expDiff + expDiffInv);
    float finalVel = 0.08 * finalVelParam;
    float finalVel_1 = 0.02 * finalVelParam;
    Vec3<float> coordinateTransformation(pfX, pfY, 0);
    coordinateTransformation = seResult.rBody * coordinateTransformation;
    if (legIndex == 0 || legIndex == 3)
      coordinateTransformation(0) += (seResult.vBody[1] > 0 ? 1 : -1) * finalVel;
    else
      coordinateTransformation(0) -= (seResult.vBody[1] > 0 ? 1 : -1) * finalVel;
    coordinateTransformation(1) += (seResult.vBody[1] > 0 ? 1 : -1) * finalVel_1;
    coordinateTransformation = seResult.rBody.transpose() * coordinateTransformation;
    pfX = coordinateTransformation(0);
    pfY = coordinateTransformation(1);
}
/// Add End
