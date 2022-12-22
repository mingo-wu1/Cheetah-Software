#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"


class Gait {
public:
  virtual ~Gait() = default;

  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() { }

  /// Add Begin by wuchunming, 20210712, collision detection
  virtual Eigen::Array4f getStancePhase() = 0;
  virtual int* getMpcTable(int horizonlength, int currentIteration, int iterationsBetweenMPC, int iterationsBetweenGaitSeg) = 0;
  /// Add End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  virtual void setTransition(bool input) = 0;
  /// Add End

protected:
  std::string _name;

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  bool _startTransition = false;
  int _startIteration = -1;
  Eigen::Array4i _standCountdown; 
  Eigen::Array4i _standCountdownInit; 
  /// Add End 
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait();
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

  /// Mod Begin by lihao, 2020-03-01, mod getMpcTable
  int* getMpcTable(int horizonlength, int currentIteration, int iterationsBetweenMPC, int iterationsBetweenGaitSeg);
  /// Mod End

  /// Add Begin by wuchunming, 20210712, collision detection
  Eigen::Array4f getStancePhase() override {return _durationsFloat;};
  /// Add End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  void setTransition(bool input) { _startTransition = input;};
  /// Add End 

private:
  int* _mpc_table;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  Array4f _phase;
};



class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait();
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

  /// Add Begin by wuchunming, 20210712, collision detection
  Eigen::Array4f getStancePhase() override {return Array4f(_duty_cycle * 10, _duty_cycle * 10, _duty_cycle * 10, _duty_cycle* 10);};
  int* getMpcTable(int, int, int, int){return getMpcTable();};
  /// Add End

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  void setTransition(bool input) { _startTransition = input;};
  /// Add End 
  
private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif //PROJECT_GAIT_H
