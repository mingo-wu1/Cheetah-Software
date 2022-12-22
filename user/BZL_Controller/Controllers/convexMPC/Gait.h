#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"

/*
5.【步态相位分段执行部分】步态相关（根据步态相位控制器计算的实时参数作切换）
（1）步态相位控制器计算的实时参数（其他地方进行计算）
计算支撑/摆动腿状态相关参数
1）若处于支撑相阶段【防盗标记–盒子君hzj】
（1）设置足部接触状态：支撑相=1
（2）判断支撑相位切换状态
		方法：支撑相状态=每只脚的当前相位【0~1】/相切换到swing的相位【1】，若为1就可以切换到支撑相，反之
（3）计算摆动相位：摆动阶段尚未开始，因为脚处于站姿
（4）计算站姿剩余时间
		公式：站姿剩余的时间=步态总时间*（摆动相位-现在的相位）
（5）计算摆动剩余时间
 		支撑相没有摆动时间
（6）判断是不是第一次接触，设置接触标志位，处理从站立到其他步态刚开始逻辑用的

2）若处于摆动相阶段
（1）设置足部接触状态：摆动相=0
（2）判断支撑相位
		脚处于摆动状态，站立状态位0
（3）计算摆动相位
		公式：摆动相相位=（当前的相位-摆动相默认相位）/（1-摆动相默认相位）
（4）计算站姿剩余时间
		脚在摆动，没有站立时间，故站立剩余时间为0
（5）计算摆动剩余时间
		公式：挥杆剩余时间=步态总时间*（1-当前相位值）
（6）判断是不是第一次摆动，设置摆动标志位，处理从站立到其他步态刚开始逻辑用的
（2）步态相位MPC处理的原理

步态周期的计算
一个步态周期由horizonLength(10)分为10个mpc周期组成
步态按1KHz处理，mpc计数间隔为30ms左右 ，一毫秒计数一次来控制频率 即一个mpc周期为30ms
则步态周期为 10*30 =300ms

offsets是步态的相关相位参数，durations支撑时间参数，用整数即分段来计算
_offsetsFloat、_durationsFloat在0~1之间
*/
//（3）步态相位MPC处理基类
class Gait {
public:
  virtual ~Gait() = default;
  //虚函数 在之后定义
  virtual Vec4<float> getContactState() = 0;                                                                                      //（1）获得四足接触状态
  virtual Vec4<float> getSwingState() = 0;                                                                                        //（2）获得四足摆动状态
  virtual int* getMpcTable() = 0;                                                                                                 //（3）获得mpc需要的gait数组 悬空与否
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;                                                 //（4）设置MPC迭代
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;                                                                   //（5）获得这次支撑的持续时间
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;                                                                    //（6）获得这次摆动的持续时间
  virtual int getCurrentGaitPhase() = 0;                                                                                          //（7）获得当前步态所处相位0~1
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

// （4）摆动腿步态相位MPC处理子类【OffsetDurationGait】
class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait();                                                                                                           //步态周期分段数 相位差 支撑持续时间（按分段算） 步态名称
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState();                                                                                                  //同上
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
  Array4i _offsets; // offset in mpc segments                                                                                         //（按分段算）
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)                                                                                 //按百分比算
  Array4f _durationsFloat; // durations in phase (0 to 1)
  int _stance;                                                                                                                        //支撑时间，按分段算
  int _swing;                                                                                                                         //摆动时间 分段算
  int _iteration;                                                                                                                     //步态片段计数
  int _nIterations;                                                                                                                   //步态片段数
  Array4f _phase;                                                                                                                     //当前相位
};


//（5）支撑腿腿步态相位MPC处理子类【MixedFrequncyGait】
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
