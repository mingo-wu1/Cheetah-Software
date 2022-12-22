/*
 * @Author: your name
 * @Date: 2021-10-26 19:40:18
 * @LastEditTime: 2021-10-26 19:40:18
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /Cheetah_BT_test/robot/include/rt/rt_rc_interface.h
 */
/**
 * @file rt_rc_interface.h
 *
 */
#ifndef _RT_RC_INTERFACE
#define _RT_RC_INTERFACE

class rc_control_settings {
  public:
    double     mode;
    double     p_des[2]; // (x, y) -1 ~ 1
    double     height_variation; // -1 ~ 1
    double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3]; // -1 ~ 1
    double     omega_des[3]; // -1 ~ 1
    double     variable[3];
};


namespace RC_mode{
/*   constexpr int OFF = 0; */

  constexpr int STAND_UP = 1;
  constexpr int QP_STAND = 2;
  constexpr int RECOVERY_STAND = 3;
  constexpr int PRONE = 4;
  constexpr int LOCOMOTION = 5;
  constexpr int BACKFLIP = 6;
  constexpr int FRONTJUMP = 7;
  constexpr int VISION = 8;
  constexpr int JOINT_PD = 9;
  constexpr int IMPEDANCE_CONTROL = 10;

  constexpr int BACKFLIP_PRE = 11;

  // Experiment Mode
  constexpr int TWO_LEG_STANCE_PRE = 20;
  constexpr int TWO_LEG_STANCE = 21;

  constexpr int PASSIVE = 100;
};

void sbus_packet_complete();

void get_rc_control_settings(void* settings);
//void get_rc_channels(void* settings);

void* v_memcpy(void* dest, volatile void* src, size_t n);

float deadband(float command, float deadbandRegion, float minVal, float maxVal);

#endif
