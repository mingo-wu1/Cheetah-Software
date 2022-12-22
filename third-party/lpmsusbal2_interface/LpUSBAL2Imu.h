#pragma once

#include "OpenZen.h"
#include <array>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include "cTypes.h"
#include "cppTypes.h"
#include "cppTypes.h"


class LpUSBAL2Imu
{
public:
  LpUSBAL2Imu();
  ~LpUSBAL2Imu();
  bool initImu();
  bool update();
  friend void pollLoop(std::reference_wrapper<zen::ZenClient> client);
  friend void addDiscoveredSensor(const ZenEventData_SensorFound& desc);

  Vec3<float> gyro;
  Vec3<float> acc;
  Vec4<float> quat;
  Vec3<float> rpy;
private:
  void exit();
  zen::ZenClient* _client = NULL;
  std::thread _pollingThread;
  std::thread _initThread;
};