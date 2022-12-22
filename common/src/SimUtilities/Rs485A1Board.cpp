/*! @file Rs485A1Board.cpp
 *  @brief drive A1 motor code, used to simulate the A1 motor. 
 *  @author hanyuanqiang
 *  
 */

#include <stdio.h>

#include "SimUtilities/Rs485A1Board.h"

/*!
 * RS485 A1 board setup (per board)
 */
void Rs485A1Board::init(float sideSign, s32 board) {
  this->board_num = board;
  this->side_sign = sideSign;
}

/*!
 * Reset all data for the board
 */
void Rs485A1Board::resetData() {
  if (data == nullptr) {
    printf(
        "[ERROR: RS485 A1 board] reset_rs485_a1_board_data called when "
        "cheetahlcm_rs485_a1_data_t* was null\n");
    return;
  }

  data->flags[board_num] = 0;
  data->qd_abad[board_num] = 0.f;
  data->qd_hip[board_num] = 0.f;
  data->qd_knee[board_num] = 0.f;
  data->q_abad[board_num] = 0.f;
  data->q_hip[board_num] = 0.f;
  data->q_knee[board_num] = 0.f;
  data->rs485_a1_driver_status = 0;
}

/*!
 * Reset all commands for the board
 */
void Rs485A1Board::resetCommand() {
  if (cmd == nullptr) {
    printf(
        "[ERROR: RS485 A1 board] reset_rs485_a1_board_command called when "
        "cheetahlcm_rs485_a1_command_t* was null\n");
    return;
  }

  cmd->flags[board_num] = 0;
  cmd->kd_abad[board_num] = 0.f;
  cmd->kd_hip[board_num] = 0.f;
  cmd->kd_knee[board_num] = 0.f;
  cmd->kp_abad[board_num] = 0.f;
  cmd->kp_hip[board_num] = 0.f;
  cmd->kp_knee[board_num] = 0.f;
  cmd->qd_des_abad[board_num] = 0.f;
  cmd->qd_des_hip[board_num] = 0.f;
  cmd->qd_des_knee[board_num] = 0.f;
  cmd->q_des_abad[board_num] = 0.f;
  cmd->q_des_hip[board_num] = 0.f;
  cmd->q_des_knee[board_num] = 0.f;
  cmd->tau_abad_ff[board_num] = 0.f;
  cmd->tau_hip_ff[board_num] = 0.f;
  cmd->tau_hip_ff[board_num] = 0.f;
  cmd->tau_knee_ff[board_num] = 0.f;
}

/*!
 * Run RS485 A1 board control
 */
void Rs485A1Board::run() {
  iter_counter++;
  if (cmd == nullptr || data == nullptr) {
    printf(
        "[ERROR: RS485 A1 board] run_rs485_a1_board_iteration called with null "
        "command or data!\n");
    torque_out[0] = 0.f;
    torque_out[1] = 0.f;
    torque_out[2] = 0.f;
    return;
  }

  /// Check abad softstop ///
  if (data->q_abad[board_num] > q_limit_p[0]) {
    torque_out[0] = kp_softstop * (q_limit_p[0] - data->q_abad[board_num]) -
                    kd_softstop * (data->qd_abad[board_num]) +
                    cmd->tau_abad_ff[board_num];
  } else if (data->q_abad[board_num] < q_limit_n[0]) {
    torque_out[0] = kp_softstop * (q_limit_n[0] - data->q_abad[board_num]) -
                    kd_softstop * (data->qd_abad[board_num]) +
                    cmd->tau_abad_ff[board_num];
  } else {
    torque_out[0] = cmd->kp_abad[board_num] *
                        (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
                    cmd->kd_abad[board_num] * (cmd->qd_des_abad[board_num] -
                                               data->qd_abad[board_num]) +
                    cmd->tau_abad_ff[board_num];
  }

  /// Check hip softstop ///
  if (data->q_hip[board_num] > q_limit_p[1]) {
    torque_out[1] = kp_softstop * (q_limit_p[1] - data->q_hip[board_num]) -
                    kd_softstop * (data->qd_hip[board_num]) +
                    cmd->tau_hip_ff[board_num];
  } else if (data->q_hip[board_num] < q_limit_n[1]) {
    torque_out[1] = kp_softstop * (q_limit_n[1] - data->q_hip[board_num]) -
                    kd_softstop * (data->qd_hip[board_num]) +
                    cmd->tau_hip_ff[board_num];
  } else {
    torque_out[1] = cmd->kp_hip[board_num] *
                        (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
                    cmd->kd_hip[board_num] *
                        (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
                    cmd->tau_hip_ff[board_num];
  }

  /// No knee softstop right now ///
  torque_out[2] = cmd->kp_knee[board_num] *
                      (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
                  cmd->kd_knee[board_num] *
                      (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
                  cmd->tau_knee_ff[board_num];

  const float* torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }

  for (int i = 0; i < 3; i++) {
    if (torque_out[i] > torque_limits[i]) torque_out[i] = torque_limits[i];
    if (torque_out[i] < -torque_limits[i]) torque_out[i] = -torque_limits[i];
  }

}
