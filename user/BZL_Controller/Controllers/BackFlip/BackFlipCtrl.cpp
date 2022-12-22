#include "BackFlipCtrl.hpp"


template <typename T>
BackFlipCtrl<T>::BackFlipCtrl(DataReader* data_reader,float _dt) : DataReadCtrl<T>(data_reader, _dt) {}


template <typename T>
BackFlipCtrl<T>::~BackFlipCtrl() {}

template <typename T>
void BackFlipCtrl<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) {

  /// Mod Begin by wuchunming, 2021-05-20, add Finish Flag and fix backflip bug;
  if(false == b_preparation){
    DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;
  }else{
    _curr_time = 0;
  }
  /// origin code
  /// DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;
  /// Mod End

  DataCtrl::_b_Preparation = b_preparation;
  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];
      command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx] + 0 * _curr_time;
      command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];
      /// Mod Begin by peibo zhaoyudong, 2020-04-06, Add the preparation operation in the back flip mode 
      // and extract the key parameters to the parameter table
      command[leg].kpJoint(jidx, jidx) = leg < 2 ? Kp_backfilp_f[jidx] : Kp_backfilp_b[jidx];
      command[leg].kdJoint(jidx, jidx) = leg < 2 ? Kd_backfilp_f[jidx] : Kd_backfilp_b[jidx];
      /// Ori Code:
      // command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];
      // command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];
      /// Mod End
    }
  }
}

template <typename T>
void BackFlipCtrl<T>::_update_joint_command() {
  int pre_mode_duration(2000);
  /// Mod Begin by peibo zhaoyudong,  2020-04-06, Add the preparation operation in the back flip mode 
  //and extract the key parameters to the parameter table
  int tuck_iteration = (int)backfilp_tuck_iteration;
  int ramp_end_iteration = (int)backfilp_ramp_end_iteration;

  Kp_backfilp_f = { (T)Kp_backfilp_step1(0),(T)Kp_backfilp_step1(1),(T)Kp_backfilp_step1(2) };
  Kp_backfilp_b = { (T)Kp_backfilp_step1(0),(T)Kp_backfilp_step1(1),(T)Kp_backfilp_step1(2) };
  Kd_backfilp_f = { (T)Kd_backfilp_step1(0),(T)Kd_backfilp_step1(1),(T)Kd_backfilp_step1(2) };
  Kd_backfilp_b = { (T)Kd_backfilp_step1(0),(T)Kd_backfilp_step1(1),(T)Kd_backfilp_step1(2) };
  /// Ori Code:
  // int tuck_iteration(600);
  // int ramp_end_iteration(650);

  /// Mod Begin by wuchunming, 2021-04-29, mod Kp_joint for MIT backflip
  // this->_Kp_joint = {15.0, 15.0, 15.0};
  // origin code
  // this->_Kp_joint = {10.0, 10.0, 10.0};
  /// Mod End
  // this->_Kd_joint = {1.0, 1.0, 1.0};
  /// Mod End

  float tau_mult;

  DataCtrl::_des_jpos.setZero();
  DataCtrl::_des_jvel.setZero();
  DataCtrl::_jtorque.setZero();

  if ( (DataCtrl::pre_mode_count <  pre_mode_duration) || DataCtrl::_b_Preparation) {  
    // move to the initial configuration to prepare for
    // backfliping
    if (DataCtrl::pre_mode_count == 0) {
      printf("plan_timesteps: %d \n", DataCtrl::_data_reader->plan_timesteps);
    }
    // printf("pre_mode_count: %d \n", pre_mode_count);

    DataCtrl::pre_mode_count += DataCtrl::_key_pt_step;
    DataCtrl::current_iteration = 0;
    tau_mult = 0;
  } else {
    /// Mod Begin by peibo zhaoyudong, 2020-04-06, Add the preparation operation in the back flip mode 
    //and extract the key parameters to the parameter table
    tau_mult = backfilp_tau_mult;
    /// ori code:
    //tau_mult = 1.2;
    /// Mod End
    // tau_mult = 1.;
  }

  if (DataCtrl::current_iteration > DataCtrl::_data_reader->plan_timesteps - 1) {
    DataCtrl::current_iteration = DataCtrl::_data_reader->plan_timesteps - 1;
  }

  float* current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration);
  float* tau = current_step + tau_offset;

  Vec3<float> q_des_front;
  Vec3<float> q_des_rear;
  Vec3<float> qd_des_front;
  Vec3<float> qd_des_rear;
  Vec3<float> tau_front;
  Vec3<float> tau_rear;

  q_des_front << 0.0, current_step[3], current_step[4];
  q_des_rear << 0.0, current_step[5], current_step[6];
  qd_des_front << 0.0, current_step[10], current_step[11];
  qd_des_rear << 0.0, current_step[12], current_step[13];
  /// Mod Begin by zhaoyudong peibo,2021-04-30,The moment coefficients of front and rear legs are controlled separately in the backflip mode
  // if (tau_mult > 0)
  // {
	//   tau_front << 0.0, 1.7 * tau[0] / 2.0, 1.7 * tau[1] / 2.0;
  // }
  // else
  // {
	//   tau_front << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
  // }
  /// Ori Code:
  tau_front << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
  /// Mod End;
  tau_rear << 0.0, tau_mult * tau[2] / 2.0, tau_mult * tau[3] / 2.0;
	/// Add Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
	Vec3<float> tau_fr, tau_fl, tau_br, tau_bl;
	if (tau_mult > 0)
	{
		tau_fr << 0.0, backfilp_tau_mult_fr * tau[0] / 2.0, backfilp_tau_mult_fr * tau[1] / 2.0;
		tau_fl << 0.0, backfilp_tau_mult_fl * tau[0] / 2.0, backfilp_tau_mult_fl * tau[1] / 2.0;
		tau_br << 0.0, backfilp_tau_mult_br * tau[2] / 2.0, backfilp_tau_mult_br * tau[3] / 2.0;
		tau_bl << 0.0, backfilp_tau_mult_bl * tau[2] / 2.0, backfilp_tau_mult_bl * tau[3] / 2.0;
	}
	else
	{
		tau_fr << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
		tau_fl << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
		tau_br << 0.0, tau_mult * tau[2] / 2.0, tau_mult * tau[3] / 2.0;
		tau_bl << 0.0, tau_mult * tau[2] / 2.0, tau_mult * tau[3] / 2.0;
	}
	/// Add End
  //pretty_print(tau_front, std::cout, "tau front");
  //pretty_print(tau_rear, std::cout, "tau rear");
  float s(0.);

  if (DataCtrl::current_iteration >= tuck_iteration) {  // ramp to landing configuration
    qd_des_front << 0.0, 0.0, 0.0;
    qd_des_rear << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
    tau_rear << 0.0, 0.0, 0.0;
    /// Add Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
    tau_fr << 0.0, 0.0, 0.0;
    tau_fl << 0.0, 0.0, 0.0;
    tau_br << 0.0, 0.0, 0.0;
    tau_bl << 0.0, 0.0, 0.0;
    /// Add End

    s = (float)(DataCtrl::current_iteration - tuck_iteration) /
      (ramp_end_iteration - tuck_iteration);

    if (s > 1) {
      s = 1;
    }

    Vec3<float> q_des_front_0;
    Vec3<float> q_des_rear_0;
    Vec3<float> q_des_front_f;
    Vec3<float> q_des_rear_f;

    current_step = DataCtrl::_data_reader->get_plan_at_time(tuck_iteration);
    q_des_front_0 << 0.0, current_step[3], current_step[4];
    q_des_rear_0 << 0.0, current_step[5], current_step[6];

    current_step = DataCtrl::_data_reader->get_plan_at_time(0);
    // q_des_front_f << 0.0, current_step[3], current_step[4];
    // q_des_rear_f << 0.0, current_step[5], current_step[6];

    //q_des_front_f << 0.0, -0.8425, 1.65;
    //q_des_rear_f << 0.0, -0.8425, 1.65;

    // DH
    q_des_front_f << 0.0, -0.8425, 1.70;
    q_des_rear_f << 0.0, -1.0525, 1.65;


    q_des_front = (1 - s) * q_des_front_0 + s * q_des_front_f;
    q_des_rear = (1 - s) * q_des_rear_0 + s * q_des_rear_f;
    /// Mod Begin by peibo zhaoyudong, 2020-04-06, Add the preparation operation in the back flip mode 
    //and extract the key parameters to the parameter table
    Kp_backfilp_f = { (T)Kp_backfilp_step2_f(0),(T)Kp_backfilp_step2_f(1),(T)Kp_backfilp_step2_f(2) };
    Kd_backfilp_f = { (T)Kd_backfilp_step2_f(0),(T)Kd_backfilp_step2_f(1),(T)Kd_backfilp_step2_f(2) };
    Kp_backfilp_b = { (T)Kp_backfilp_step2_b(0),(T)Kp_backfilp_step2_b(1),(T)Kp_backfilp_step2_b(2) };
    Kd_backfilp_b = { (T)Kd_backfilp_step2_b(0),(T)Kd_backfilp_step2_b(1),(T)Kd_backfilp_step2_b(2) };
    /// Ori Code:
    // this->_Kp_joint = {25.0, 25.0, 25.0};
    // this->_Kd_joint = {1.5, 1.5, 1.5};
    /// Mod End

  }

  /// Mod Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
	// Abduction
	for (int i = 0; i < 12; i += 3) {
		DataCtrl::_des_jpos[i] = 0.0;
		DataCtrl::_des_jvel[i] = 0.0;
		DataCtrl::_jtorque[i] = 0.0;
	}
	DataCtrl::_des_jpos[0] = s * (-0.2);
	DataCtrl::_des_jpos[3] = s * (0.2);
	DataCtrl::_des_jpos[6] = s * (-0.2);
	DataCtrl::_des_jpos[9] = s * (0.2);

	// Front Hip
	for (int i = 1; i < 6; i += 3) {
		DataCtrl::_des_jpos[i] = q_des_front[1];
		DataCtrl::_des_jvel[i] = qd_des_front[1];
	}
	DataCtrl::_jtorque[1] = tau_fr[1];
	DataCtrl::_jtorque[4] = tau_fl[1];

	// Front Knee
	for (int i = 2; i < 6; i += 3) {
		DataCtrl::_des_jpos[i] = q_des_front[2];
		DataCtrl::_des_jvel[i] = qd_des_front[2];
	}
	DataCtrl::_jtorque[2] = tau_fr[2];
	DataCtrl::_jtorque[5] = tau_fl[2];
	// Hind Hip
	for (int i = 7; i < 12; i += 3) {
		DataCtrl::_des_jpos[i] = q_des_rear[1];
		DataCtrl::_des_jvel[i] = qd_des_rear[1];
	}
	DataCtrl::_jtorque[7] = tau_br[1];
	DataCtrl::_jtorque[10] = tau_bl[1];
	// Hind Knee
	for (int i = 8; i < 12; i += 3) {
		DataCtrl::_des_jpos[i] = q_des_rear[2];
		DataCtrl::_des_jvel[i] = qd_des_rear[2];
	}
	DataCtrl::_jtorque[8] = tau_br[2];
	DataCtrl::_jtorque[11] = tau_bl[2];
	/// Ori Code:
	//// Abduction
	//for (int i = 0; i < 12; i += 3) {
	//  DataCtrl::_des_jpos[i] = 0.0;
	//  DataCtrl::_des_jvel[i] = 0.0;
	//  DataCtrl::_jtorque[i] = 0.0;
	//}
	//DataCtrl::_des_jpos[0] = s * (-0.2);
	//DataCtrl::_des_jpos[3] = s * (0.2);
	//DataCtrl::_des_jpos[6] = s * (-0.2);
	//DataCtrl::_des_jpos[9] = s * (0.2);

	//// Front Hip
	//for (int i = 1; i < 6; i += 3) {
	//  DataCtrl::_des_jpos[i] = q_des_front[1];
	//  DataCtrl::_des_jvel[i] = qd_des_front[1];
	//  DataCtrl::_jtorque[i] = tau_front[1];
	//}

	//// Front Knee
	//for (int i = 2; i < 6; i += 3) {
	//  DataCtrl::_des_jpos[i] = q_des_front[2];
	//  DataCtrl::_des_jvel[i] = qd_des_front[2];
	//  DataCtrl::_jtorque[i] = tau_front[2];
	//}

	//// Hind Hip
	//for (int i = 7; i < 12; i += 3) {
	//  DataCtrl::_des_jpos[i] = q_des_rear[1];
	//  DataCtrl::_des_jvel[i] = qd_des_rear[1];
	//  DataCtrl::_jtorque[i] = tau_rear[1];
	//}

	//// Hind Knee
	//for (int i = 8; i < 12; i += 3) {
	//  DataCtrl::_des_jpos[i] = q_des_rear[2];
	//  DataCtrl::_des_jvel[i] = qd_des_rear[2];
	//  DataCtrl::_jtorque[i] = tau_rear[2];
	//}
	/// Mod End

  // Update rate 0.5kHz
  DataCtrl::current_iteration += DataCtrl::_key_pt_step;
}

template class BackFlipCtrl<double>;
template class BackFlipCtrl<float>;
