#ifndef PROJECT_LPIG1IMU_H
#define PROJECT_LPIG1IMU_H

#include <string>
#include <mutex>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <iostream>

#include "../../../common/include/cTypes.h"
#include "../../../common/include/cppTypes.h"

#include "lpmsig1/LpmsIG1I.h"
#include "lpmsig1/SensorDataI.h"
#include "lpmsig1/LpmsIG1Registers.h"

// #include "../../lcm-types/cpp/lpig1_lcmt.hpp"

struct IG1Command
{
  short command;
  union Data
  {
    uint32_t i[64];
    float f[64];
    unsigned char c[256];
  } data;
  int dataLength;
};

class LpIG1Imu
{
public:
  LpIG1Imu();

  ~LpIG1Imu();

  void init(u32 port, u32 baud_rate);

  bool tryInit(u32 port, u32 baud_rate);

  bool setCommandMode();

  void cmdGotoCommandMode();

  bool setStreamingMode();

  void cmdGotoStreamingMode();

  void mode_setup();

  void get_device_info();
  
  void setup_streaming();
  
  void enable();
  
  void run();

  bool getImuData();

  void cmdGetImuData();
  
  void update();

  // void self_test();
  // void basic_report();
  // void print_packet_stats();
  // void zero_gyro();
  // void updateLCM(lpig1_lcmt* message);

  u32 invalid_packets = 0;
  u32 timeout_packets = 0;
  u32 unknown_packets = 0;
  u32 good_packets = 0;

  Vec3<float> gyro;
  Vec3<float> acc;
  Vec4<float> quat;

private:
  IG1I *sensor1;
};

#endif //PROJECT_LPIG1IMU_H
