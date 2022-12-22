#ifndef BZL_USERPARAMETERS_H
#define BZL_USERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class BZL_UserParameters : public ControlParameters {
public:
  BZL_UserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(cmpc_gait),
        INIT_PARAMETER(cmpc_x_drag),
        INIT_PARAMETER(cmpc_use_sparse),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(cmpc_bonus_swing),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        //INIT_PARAMETER(Kp_joint_swing),
        //INIT_PARAMETER(Kd_joint_swing),
        INIT_PARAMETER(Q_pos),
        INIT_PARAMETER(Q_vel),
        INIT_PARAMETER(Q_ori),
        INIT_PARAMETER(Q_ang),
        INIT_PARAMETER(R_control),
        INIT_PARAMETER(R_prev),
        INIT_PARAMETER(two_leg_orient),
        INIT_PARAMETER(stance_legs),
        INIT_PARAMETER(use_jcqp),
        INIT_PARAMETER(jcqp_max_iter),
        INIT_PARAMETER(jcqp_rho),
        INIT_PARAMETER(jcqp_sigma),
        INIT_PARAMETER(jcqp_alpha),
        INIT_PARAMETER(jcqp_terminate),
        INIT_PARAMETER(Swing_Kp_cartesian),
        INIT_PARAMETER(Swing_Kd_cartesian),
        INIT_PARAMETER(Swing_Kp_joint),
        INIT_PARAMETER(Swing_Kd_joint),
        INIT_PARAMETER(Swing_step_offset),
        INIT_PARAMETER(Swing_traj_height),
        INIT_PARAMETER(Swing_use_tau_ff),
        INIT_PARAMETER(RPC_Q_p),
        INIT_PARAMETER(RPC_Q_theta),
        INIT_PARAMETER(RPC_Q_dp),
        INIT_PARAMETER(RPC_Q_dtheta),
        INIT_PARAMETER(RPC_R_r),
        INIT_PARAMETER(RPC_R_f),
        INIT_PARAMETER(RPC_H_r_trans),
        INIT_PARAMETER(RPC_H_r_rot),
        INIT_PARAMETER(RPC_H_theta0),
        INIT_PARAMETER(RPC_H_phi0),
        INIT_PARAMETER(RPC_mass),
        INIT_PARAMETER(RPC_inertia),
        INIT_PARAMETER(RPC_gravity),
        INIT_PARAMETER(RPC_mu),
        INIT_PARAMETER(RPC_filter),
        INIT_PARAMETER(RPC_use_pred_comp),
        INIT_PARAMETER(RPC_use_async_filt),
        INIT_PARAMETER(RPC_visualize_pred),
        INIT_PARAMETER(RPC_use_separate),
        INIT_PARAMETER(des_p),
        INIT_PARAMETER(des_theta),
        INIT_PARAMETER(des_dp),
        INIT_PARAMETER(des_dtheta),
        INIT_PARAMETER(des_theta_max),
        INIT_PARAMETER(des_dp_max),
        INIT_PARAMETER(des_dtheta_max),
        INIT_PARAMETER(gait_type),
        INIT_PARAMETER(gait_period_time),
        INIT_PARAMETER(gait_switching_phase),
        INIT_PARAMETER(gait_override),
        INIT_PARAMETER(gait_max_leg_angle),
        INIT_PARAMETER(gait_max_stance_time),
        INIT_PARAMETER(gait_min_stance_time),

        /// Add Begin by wuchunming, 2021-03-10, add BZL Parameters
        // BZL parameters
        INIT_PARAMETER(use_body_pitch_compute),        
        INIT_PARAMETER(gait_seg),
        INIT_PARAMETER(iterations_between_gait_seg),
        INIT_PARAMETER(foot_height),
        INIT_PARAMETER(raibert_coeff),
        INIT_PARAMETER(factor_of_iterations_between_mpc),
        INIT_PARAMETER(horizon_length),
        INIT_PARAMETER(alpha),
        INIT_PARAMETER(mu),
        INIT_PARAMETER(fmax),
        INIT_PARAMETER(w_floating1),
        INIT_PARAMETER(w_floating2),
        INIT_PARAMETER(w_floating3),
        INIT_PARAMETER(w_floating4),
        INIT_PARAMETER(w_floating5),
        INIT_PARAMETER(w_floating6),
        INIT_PARAMETER(w_rf1),
        INIT_PARAMETER(w_rf2),
        INIT_PARAMETER(w_rf3),
        INIT_PARAMETER(w_rf4),
        INIT_PARAMETER(w_rf5),
        INIT_PARAMETER(w_rf6),
        INIT_PARAMETER(w_rf7),
        INIT_PARAMETER(w_rf8),
        INIT_PARAMETER(w_rf9),
        INIT_PARAMETER(w_rf10),
        INIT_PARAMETER(w_rf11),
        INIT_PARAMETER(w_rf12),
        INIT_PARAMETER(Kp_joint_ex1),
        INIT_PARAMETER(Kp_joint_ex2),
        INIT_PARAMETER(Kp_joint_ex3),
        INIT_PARAMETER(Kp_joint_ex4),
        INIT_PARAMETER(Kd_joint_ex1),
        INIT_PARAMETER(Kd_joint_ex2),
        INIT_PARAMETER(Kd_joint_ex3),
        INIT_PARAMETER(Kd_joint_ex4),
        INIT_PARAMETER(Q1),
        INIT_PARAMETER(Q2),
        INIT_PARAMETER(Q3),
        INIT_PARAMETER(Q4),
        INIT_PARAMETER(weights1),
        INIT_PARAMETER(weights2),
        INIT_PARAMETER(weights3),
        INIT_PARAMETER(weights4),
        /// Add End
        /// Add Begin by peibo, 2021-03-29,automatically change kp,kd parameters with speed
        INIT_PARAMETER(cur_speed_level),
        INIT_PARAMETER(speed_threshold_value_1),
        INIT_PARAMETER(speed_threshold_value_2),

        INIT_PARAMETER(Kp_joint_fr_heighSpeed_1),
        INIT_PARAMETER(Kd_joint_fr_heighSpeed_1),
        INIT_PARAMETER(Kp_joint_fl_heighSpeed_1),
        INIT_PARAMETER(Kd_joint_fl_heighSpeed_1),
        INIT_PARAMETER(Kp_joint_br_heighSpeed_1),
        INIT_PARAMETER(Kd_joint_br_heighSpeed_1),
        INIT_PARAMETER(Kp_joint_bl_heighSpeed_1),
        INIT_PARAMETER(Kd_joint_bl_heighSpeed_1),

        INIT_PARAMETER(Kp_joint_fr_heighSpeed_2),
        INIT_PARAMETER(Kd_joint_fr_heighSpeed_2),
        INIT_PARAMETER(Kp_joint_fl_heighSpeed_2),
        INIT_PARAMETER(Kd_joint_fl_heighSpeed_2),
        INIT_PARAMETER(Kp_joint_br_heighSpeed_2),
        INIT_PARAMETER(Kd_joint_br_heighSpeed_2),
        INIT_PARAMETER(Kp_joint_bl_heighSpeed_2),
        INIT_PARAMETER(Kd_joint_bl_heighSpeed_2),
        /// Add End
        /// Add Begin by peibo 2021-03-31,Velocity expected smoothing
        INIT_PARAMETER(liner_a_max),
        INIT_PARAMETER(liner_a_limit_enable),
        /// Add End
        /// Add Begin by peibo, zhaoyudong, 2020-04-06, Add the preparation operation in the back flip mode
        // and extract the key parameters to the parameter table
        INIT_PARAMETER(backfilp_tuck_iteration),
        INIT_PARAMETER(backfilp_ramp_end_iteration),
        INIT_PARAMETER(backfilp_tau_mult),
        INIT_PARAMETER(Kp_backfilp_step1),
        INIT_PARAMETER(Kd_backfilp_step1),
        INIT_PARAMETER(Kp_backfilp_step2_f),
        INIT_PARAMETER(Kd_backfilp_step2_f),
        INIT_PARAMETER(Kp_backfilp_step2_b),
        INIT_PARAMETER(Kd_backfilp_step2_b),
        /// Add End
        /// Add Begin by peibo, 2021-04-29,0430 integration of related functions, adding stair operation, adding WBC operation separation in different modes
        INIT_PARAMETER(cmpc_gait_stair),
        INIT_PARAMETER(iterationsBetweenGaitSeg_stair),
        INIT_PARAMETER(wbc_param_mode), //0: balance stand 1:locomotion  2:stair 
        INIT_PARAMETER(Kp_joint_fr_stair),
        INIT_PARAMETER(Kd_joint_fr_stair),
        INIT_PARAMETER(Kp_joint_fl_stair),
        INIT_PARAMETER(Kd_joint_fl_stair),
        INIT_PARAMETER(Kp_joint_br_stair),
        INIT_PARAMETER(Kd_joint_br_stair),
        INIT_PARAMETER(Kp_joint_bl_stair),
        INIT_PARAMETER(Kd_joint_bl_stair),

        INIT_PARAMETER(Kp_joint_fr_balanceStand),
        INIT_PARAMETER(Kd_joint_fr_balanceStand),
        INIT_PARAMETER(Kp_joint_fl_balanceStand),
        INIT_PARAMETER(Kd_joint_fl_balanceStand),
        INIT_PARAMETER(Kp_joint_br_balanceStand),
        INIT_PARAMETER(Kd_joint_br_balanceStand),
        INIT_PARAMETER(Kp_joint_bl_balanceStand),
        INIT_PARAMETER(Kd_joint_bl_balanceStand),
        /// Add End
        /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
        INIT_PARAMETER(Kp_joint_fr_bounding_gait),
        INIT_PARAMETER(Kd_joint_fr_bounding_gait),
        INIT_PARAMETER(Kp_joint_fl_bounding_gait),
        INIT_PARAMETER(Kd_joint_fl_bounding_gait),
        INIT_PARAMETER(Kp_joint_br_bounding_gait),
        INIT_PARAMETER(Kd_joint_br_bounding_gait),
        INIT_PARAMETER(Kp_joint_bl_bounding_gait),
        INIT_PARAMETER(Kd_joint_bl_bounding_gait),
      
        INIT_PARAMETER(Kp_joint_fr_wbc),
        INIT_PARAMETER(Kd_joint_fr_wbc),
        INIT_PARAMETER(Kp_joint_fl_wbc),
        INIT_PARAMETER(Kd_joint_fl_wbc),
        INIT_PARAMETER(Kp_joint_br_wbc),
        INIT_PARAMETER(Kd_joint_br_wbc),
        INIT_PARAMETER(Kp_joint_bl_wbc),
        INIT_PARAMETER(Kd_joint_bl_wbc),
        /// Add End
        /// Add Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
        INIT_PARAMETER(backfilp_tau_mult_fr),
        INIT_PARAMETER(backfilp_tau_mult_fl),
        INIT_PARAMETER(backfilp_tau_mult_br),
        INIT_PARAMETER(backfilp_tau_mult_bl),
        /// Add End
        /// Add Begin by peibo, 2021-05-13,add the parameter of leg raising height in stair mode
        INIT_PARAMETER(foot_height_stair)
        /// Add End

  {}

  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, cmpc_x_drag);
  DECLARE_PARAMETER(double, cmpc_use_sparse);
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(double, cmpc_bonus_swing);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  DECLARE_PARAMETER(Vec3<double>, Q_pos);
  DECLARE_PARAMETER(Vec3<double>, Q_vel);
  DECLARE_PARAMETER(Vec3<double>, Q_ori);
  DECLARE_PARAMETER(Vec3<double>, Q_ang);
  DECLARE_PARAMETER(double, R_control);
  DECLARE_PARAMETER(double, R_prev);
  DECLARE_PARAMETER(Vec3<double>, two_leg_orient);
  DECLARE_PARAMETER(double, stance_legs);

  //DECLARE_PARAMETER(Vec3<double>, Kp_joint_swing);
  //DECLARE_PARAMETER(Vec3<double>, Kd_joint_swing);

  DECLARE_PARAMETER(double, use_jcqp);
  DECLARE_PARAMETER(double, jcqp_max_iter);
  DECLARE_PARAMETER(double, jcqp_rho);
  DECLARE_PARAMETER(double, jcqp_sigma);
  DECLARE_PARAMETER(double, jcqp_alpha);
  DECLARE_PARAMETER(double, jcqp_terminate);

  // Swing leg parameters
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_step_offset);
  DECLARE_PARAMETER(double, Swing_traj_height);
  DECLARE_PARAMETER(double, Swing_use_tau_ff);


  // Parameters used for RPC
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_p);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_theta);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_dp);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_dtheta);
  DECLARE_PARAMETER(Vec3<double>, RPC_R_r);
  DECLARE_PARAMETER(Vec3<double>, RPC_R_f);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_r_trans);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_r_rot);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_theta0);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_phi0);
  DECLARE_PARAMETER(double, RPC_mass);
  DECLARE_PARAMETER(Vec3<double>, RPC_inertia);
  DECLARE_PARAMETER(Vec3<double>, RPC_gravity);
  DECLARE_PARAMETER(double, RPC_mu);
  DECLARE_PARAMETER(Vec3<double>, RPC_filter);
  DECLARE_PARAMETER(double, RPC_use_pred_comp);
  DECLARE_PARAMETER(double, RPC_use_async_filt);
  DECLARE_PARAMETER(double, RPC_visualize_pred);
  DECLARE_PARAMETER(double, RPC_use_separate);

  // Desired states
  DECLARE_PARAMETER(Vec3<double>, des_p);
  DECLARE_PARAMETER(Vec3<double>, des_theta);
  DECLARE_PARAMETER(Vec3<double>, des_dp);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta);
  DECLARE_PARAMETER(Vec3<double>, des_theta_max);
  DECLARE_PARAMETER(Vec3<double>, des_dp_max);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta_max);

  // Gait Scheduler
  DECLARE_PARAMETER(double, gait_type);
  DECLARE_PARAMETER(double, gait_period_time);
  DECLARE_PARAMETER(double, gait_switching_phase);
  DECLARE_PARAMETER(double, gait_override);
  DECLARE_PARAMETER(double, gait_max_leg_angle);
  DECLARE_PARAMETER(double, gait_max_stance_time);
  DECLARE_PARAMETER(double, gait_min_stance_time);

  /// Add Begin by wuchunming, 2021-03-10, add BZL Parameters
  // BZL parameters
  DECLARE_PARAMETER(double, use_body_pitch_compute);
  DECLARE_PARAMETER(double, gait_seg);
  DECLARE_PARAMETER(double, iterations_between_gait_seg);
  DECLARE_PARAMETER(double, foot_height);
  DECLARE_PARAMETER(double, raibert_coeff);
  DECLARE_PARAMETER(double, factor_of_iterations_between_mpc);
  DECLARE_PARAMETER(double, horizon_length);
  DECLARE_PARAMETER(double, alpha);
  DECLARE_PARAMETER(double, mu);
  DECLARE_PARAMETER(double, fmax);
  DECLARE_PARAMETER(double, w_floating1);
  DECLARE_PARAMETER(double, w_floating2);
  DECLARE_PARAMETER(double, w_floating3);
  DECLARE_PARAMETER(double, w_floating4);
  DECLARE_PARAMETER(double, w_floating5);
  DECLARE_PARAMETER(double, w_floating6);
  DECLARE_PARAMETER(double, w_rf1);
  DECLARE_PARAMETER(double, w_rf2);
  DECLARE_PARAMETER(double, w_rf3);
  DECLARE_PARAMETER(double, w_rf4);
  DECLARE_PARAMETER(double, w_rf5);
  DECLARE_PARAMETER(double, w_rf6);
  DECLARE_PARAMETER(double, w_rf7);
  DECLARE_PARAMETER(double, w_rf8);
  DECLARE_PARAMETER(double, w_rf9);
  DECLARE_PARAMETER(double, w_rf10);
  DECLARE_PARAMETER(double, w_rf11);
  DECLARE_PARAMETER(double, w_rf12);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_ex1);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_ex2);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_ex3);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_ex4);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_ex1);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_ex2);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_ex3);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_ex4);
  DECLARE_PARAMETER(Vec3<double>, Q1);
  DECLARE_PARAMETER(Vec3<double>, Q2);
  DECLARE_PARAMETER(Vec3<double>, Q3);
  DECLARE_PARAMETER(Vec3<double>, Q4);
  DECLARE_PARAMETER(Vec3<double>, weights1);
  DECLARE_PARAMETER(Vec3<double>, weights2);
  DECLARE_PARAMETER(Vec3<double>, weights3);
  DECLARE_PARAMETER(Vec3<double>, weights4);
  /// Add End
  /// Add Begin by peibo, 2021-03-29,automatically change kp,kd parameters with speed
  DECLARE_PARAMETER(double, cur_speed_level);
  DECLARE_PARAMETER(double, speed_threshold_value_1);
  DECLARE_PARAMETER(double, speed_threshold_value_2);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_heighSpeed_1);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_heighSpeed_2);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_heighSpeed_2);
  /// Add End
  /// Add Begin by peibo, 2021-03-31,Velocity expected smoothing
  DECLARE_PARAMETER(double, liner_a_max);
  DECLARE_PARAMETER(double, liner_a_limit_enable);
  /// Add End
  /// Add Begin by peibo zhaoyudong, 2020-04-06, Add the preparation operation in the back flip mode
  //  and extract the key parameters to the parameter table
  DECLARE_PARAMETER(double, backfilp_tuck_iteration);
  DECLARE_PARAMETER(double, backfilp_ramp_end_iteration);
  DECLARE_PARAMETER(double, backfilp_tau_mult);
  DECLARE_PARAMETER(Vec3<double>, Kp_backfilp_step1);
  DECLARE_PARAMETER(Vec3<double>, Kd_backfilp_step1);
  DECLARE_PARAMETER(Vec3<double>, Kp_backfilp_step2_f);
  DECLARE_PARAMETER(Vec3<double>, Kd_backfilp_step2_f);
  DECLARE_PARAMETER(Vec3<double>, Kp_backfilp_step2_b);
  DECLARE_PARAMETER(Vec3<double>, Kd_backfilp_step2_b);
  /// Add End
  /// Add Begin by peibo 2021-04-29,0430 integration of related functions, adding stair operation, adding WBC operation separation in different modes
  DECLARE_PARAMETER(double, cmpc_gait_stair);
  DECLARE_PARAMETER(double, iterationsBetweenGaitSeg_stair);
  DECLARE_PARAMETER(double, wbc_param_mode); //0: balance stand 1:locomotion  2:stair 
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_stair);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_stair);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_stair);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_stair);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_stair);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_stair);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_stair);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_stair);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_balanceStand);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_balanceStand);
  /// Add End
  /// Add Begin by peibo, 2021-05-11,add parameters corresponding to different gait and modify the assignment method of WBC parameters
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_bounding_gait);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_bounding_gait);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fr_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fr_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_fl_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_fl_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_br_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_br_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint_bl_wbc);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint_bl_wbc);
  /// Add End
  /// Add Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
  DECLARE_PARAMETER(double, backfilp_tau_mult_fr);
  DECLARE_PARAMETER(double, backfilp_tau_mult_fl);
  DECLARE_PARAMETER(double, backfilp_tau_mult_br);
  DECLARE_PARAMETER(double, backfilp_tau_mult_bl);
  /// Add End
  /// Add Begin by peibo,2021-05-13,add the parameter of leg raising height in stair mode
  DECLARE_PARAMETER(double, foot_height_stair);
  /// Add End
};

#endif //BZL_USERPARAMETERS_H
