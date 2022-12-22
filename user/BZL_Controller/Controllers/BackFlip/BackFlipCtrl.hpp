#ifndef BACKFLIP_CTRL
#define BACKFLIP_CTRL

#include "DataReader.hpp"
#include "DataReadCtrl.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>

template <typename T>
class BackFlipCtrl : public DataReadCtrl<T> {
 public:
  BackFlipCtrl(DataReader*, float _dt);
  virtual ~BackFlipCtrl();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);
  /// Add Begin by peibo zhaoyudong,  2020-04-06,Add the preparation operation in the back flip mode 
  //and extract the key parameters to the parameter table
  double backfilp_tuck_iteration = 600;
  double backfilp_ramp_end_iteration = 650;
  double backfilp_tau_mult = 1.2;
  Vec3<double> Kp_backfilp_step1 = { 10.0, 10.0, 10.0 };
  Vec3<double> Kd_backfilp_step1 = { 1.0, 1.0, 1.0 };
  Vec3<double> Kp_backfilp_step2_f = { 25.0, 25.0, 25.0 };
  Vec3<double> Kd_backfilp_step2_f = { 1.5, 1.5, 1.5 };
  Vec3<double> Kp_backfilp_step2_b = { 25.0, 25.0, 25.0 };
  Vec3<double> Kd_backfilp_step2_b = { 1.5, 1.5, 1.5 };
  std::vector<T> Kp_backfilp_f = { 25.0, 25.0, 25.0 };
  std::vector<T> Kd_backfilp_f = { 1.5, 1.5, 1.5 };
  std::vector<T> Kp_backfilp_b = { 25.0, 25.0, 25.0 };
  std::vector<T> Kd_backfilp_b = { 1.5, 1.5, 1.5 };
  /// Add End
  /// Add Begin by peibo, zhaoyudong, 2021-05-13,the control torque coefficients of four legs in backflip mode are separated
  double backfilp_tau_mult_fr = 1.2;
  double backfilp_tau_mult_fl = 1.2;
  double backfilp_tau_mult_br = 1.2;
  double backfilp_tau_mult_bl = 1.2;
  /// Add End

 protected:
  void _update_joint_command();

};

#endif
