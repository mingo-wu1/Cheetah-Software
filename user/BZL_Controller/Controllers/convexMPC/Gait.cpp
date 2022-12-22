#include "Gait.h"

OffsetDurationGait::OffsetDurationGait()
{
  int nSegment = 10;
  Vec4<int> offsets(0,5,5,0);
  Vec4<int> durations(6,6,6,6);
  _offsets = Vec4<int>(0,5,5,0).array();
  _durations = Vec4<int>(6,6,6,6).array();
  _nIterations = nSegment;
  _name = "Trotting";
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
  _iteration = 0;
  _phase.setZero();
}

//（1）摆动步态构造函数，按分段算  功能：偏移、连续的步态构造函数，按分段算
// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),                                                                                                                  //相位差
  _durations(durations.array()),                                                                                                              //支撑时间
  _nIterations(nSegment)                                                                                                                      //步态周期分段数
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];                                                                                                         //为MPC步态表分配内存

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;                                                                                   //对应步态的腿之间相位差
  _durationsFloat = durations.cast<float>() / (float) nSegment;                                                                               //对应步态支撑持续时间在整个周期百分比

  _stance = durations[0];                                                                                                                     //支撑持续时间 用整个过程的分段计数
  _swing = nSegment - durations[0];                                                                                                           //摆动持续时间 同上
  _iteration = 0;
  _phase.setZero();
}

MixedFrequncyGait::MixedFrequncyGait()
{
  int nSegment = 10;
  Vec4<int> periods(9,13,13,9);
  float duty_cycle = 0.4; 
  _name = "Flying nine thirteenths trot";
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero(); 
}

//（1）支撑步态构造函数，按百分比算
MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string &name) {
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

MixedFrequncyGait::~MixedFrequncyGait() {
  delete[] _mpc_table;
}

//（2）摆动腿判断获取接触状态
Vec4<float> OffsetDurationGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;                                                                              //progress每条腿在整个步态周期的位置 offest是相位差补偿

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])                                                                                  //相位大于支撑结束相位，非支撑状态
    {
      /// Mod Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      if(_startTransition && 0 >= _standCountdown[i])
      {
        int index = 0;
        while(progress[i] > _durationsFloat[i])
        {
          progress[i] = progress[i] - _durationsFloat[i];
          if(++index > 100)
          {
            progress[i] = 0.1;
            break;
          }
        }
        progress[i] = progress[i] / _durationsFloat[i];
      }
      else                                                                                                                 //相位小于支撑结束相位，支撑状态
      {
        progress[i] = 0.;                                                                                                  
      }
      /// Ori Code:
      //progress[i] = 0.;
      /// Mod End
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];                                                                     //相位在支撑相中的百分比
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

//（2）支撑腿获取接触状态
Vec4<float> MixedFrequncyGait::getContactState() {
  Array4f progress = _phase;

  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _duty_cycle) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

// （3）获取摆动状态
Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;                                                                                               //相位大于摆动结束相位，非摆动状态
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];                                                                  //相位在摆动相中的百分比
    }    
    /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
    if(_startTransition && 0 >= _standCountdown[i])
    {
       progress[i] = 0;
    }
    /// Add End
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

//（3）获取摆动状态
Vec4<float> MixedFrequncyGait::getSwingState() {

  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) {
      progress[i] = 0;
    } else {
      progress[i] = progress[i] / swing_duration;
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

// （4）为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息
int* OffsetDurationGait::getMpcTable()
{

  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])                                                                                         //在接触时间内
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;

      /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      if(_startTransition && i >= _standCountdown[j])
      {
        _mpc_table[i*4 + j] = 1;
      }
      /// Add End

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }



  return _mpc_table;
}

/// Mod Begin by lihao, 2020-03-01, mod getMpcTable
int* OffsetDurationGait::getMpcTable(int horizonlength, int currentIteration, int iterationsBetweenMPC, int iterationsBetweenGaitSeg){
  for(int i = 0; i < horizonlength; i++){
    int iter = (currentIteration  + i * iterationsBetweenMPC) % (iterationsBetweenGaitSeg * _nIterations);
    float iterFloat = (float)iter / (float)(iterationsBetweenGaitSeg * _nIterations);
    Array4f progress = iterFloat - _offsetsFloat;
    for(int j = 0; j < 4; j++){
      if(progress[j] < 0) progress[j] += 1;
      if(progress[j] < _durationsFloat[j])
		    _mpc_table[i * 4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
      /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
      if(_startTransition && i >= _standCountdown[j])
      {
        _mpc_table[i*4 + j] = 1;
      }
      /// Add End
    }
  }
  return _mpc_table;
}
/// Mod End

// （4）为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息
int* MixedFrequncyGait::getMpcTable() {
  //printf("MPC table (%d):\n", _iteration);
  for(int i = 0; i < _nIterations; i++) {
    for(int j = 0; j < 4; j++) {
      int progress = (i + _iteration + 1) % _periods[j];  // progress
      if(progress < (_periods[j] * _duty_cycle)) {
        _mpc_table[i*4 + j] = 1;
      } else {
        _mpc_table[i*4 + j] = 0;
      }
      //printf("%d %d (%d %d) | ", _mpc_table[i*4 + j], progress, _periods[j], (int)(_periods[j] * _duty_cycle));
    }

    //printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", _mpc_table[i*4], _mpc_table[i*4 + 1], _mpc_table[i*4 + ])
    //printf("\n");
  }
  return _mpc_table;
}

// （5）设置MPC迭代的步态分段
void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;                                                              //细分为 nMPC_segments（10）个时间步 参考（2） A.Experimental Setup，当前在第几个步态分段中 0~9
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);             //当前在整个步态周期百分比 一个步态周期为 nMPC_segments（10）个mpc周期 //当前为站立重复计数用整个周期长度

  /// Add Begin by peibo, 2021-06-03,repair: when you enter the recovery standing mode, the body will shake
  if(_startTransition)
  {
    if(_startIteration < 0)
    {
      _startIteration = currentIteration;
      for(int i = 0;i < 4; i++)
      {
        _standCountdownInit[i] = -1;
        for(int ii = 0; _standCountdownInit[i] < 0 ; ii++)
        {
          int iter = (ii + _iteration) % _nIterations;
          Array4i progress = iter - _offsets;
            if(progress[i] < 0) progress[i] += _nIterations;
            if(progress[i] < _durations[i])
              _standCountdownInit[i] = ii;
        }
        _standCountdown[i] = _standCountdownInit[i];
      }
    }
    else
    {
      for(int i = 0;i < 4; i++)
      {
        int diffIt = ((currentIteration - _startIteration) / iterationsPerMPC) % _nIterations;
        _standCountdown[i]  = _standCountdownInit[i] - diffIt;
      }
    }
  }
  else
  {
    _startIteration = -1;
  }
  /// Add End
}

// （5）设置MPC迭代的步态分段
void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsBetweenMPC);// % _nIterations;                               //相当于现在步态相位时间在正MPC控制周期的占空比
  for(int i = 0; i < 4; i++) {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float) (iterationsBetweenMPC * _periods[i]);
  }

  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);

}

//（6）获取当前步态相位，摆动时用
int OffsetDurationGait::getCurrentGaitPhase() {
  return _iteration;
}

// （6）获取当前步态相位，摆动时用
int MixedFrequncyGait::getCurrentGaitPhase() {
  return 0;
}

//（7）获得摆动持续时间长度
float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  return dtMPC * _swing;
}

// （7）获得摆动持续时间长度
float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) {
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

//（8）获得站立持续时间长度
float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;
  return dtMPC * _stance;
}

//（8）获得站立持续时间长度
float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) {
  return dtMPC * _duty_cycle * _periods[leg];
}

void OffsetDurationGait::debugPrint() {

}

void MixedFrequncyGait::debugPrint() {

}