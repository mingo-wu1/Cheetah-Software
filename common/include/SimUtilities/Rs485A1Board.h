/*! @file Rs485A1Board.h
 *  @brief drive A1 motor code, used to simulate the A1 motor. 
 *  @author hanyuanqiang
 *  
 */

#ifndef PROJECT_RS485A1BOARD_H
#define PROJECT_RS485A1BOARD_H

#include "cTypes.h"

/*!
 * Command to A1 motor
 */
struct Rs485A1Command {
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];

  int32_t flags[4];
};

/*!
 * Data from A1 motor
 */
struct Rs485A1Data {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  float tau_abad[4];
  float tau_hip[4];
  float tau_knee[4];
  float acc_abad[4];
  float acc_hip[4];
  float acc_knee[4];
  float temp_abad[4];
  float temp_hip[4];
  float temp_knee[4];
  int32_t flags[4];
  int32_t rs485_a1_driver_status;
};

/*!
 * drive A1 motor control logic
 */
class Rs485A1Board {
 public:
  Rs485A1Board() {}
  void init(float side_sign, s32 board);
  void run();
  void resetData();
  void resetCommand();
  Rs485A1Command* cmd = nullptr;
  Rs485A1Data* data = nullptr;
  float torque_out[3];

 private:
  float side_sign;
  s32 board_num;
  const float max_torque[3] = {33.5f, 33.5f, 33.5f};  // TODO CHECK WITH BEN
  const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
  const float disabled_torque[3] = {0.f, 0.f, 0.f};
  const float q_limit_p[3] = {1.5f, 5.0f, 0.f};
  const float q_limit_n[3] = {-1.5f, -5.0f, 0.f};
  const float kp_softstop = 16.f;
  const float kd_softstop = 32.f;
  s32 iter_counter = 0;
};



#endif  // PROJECT_RS485A1BOARD_H