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

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
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

Vec4<float> OffsetDurationGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
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
      else
      {
        progress[i] = 0.;
      }
      /// Ori Code:
      //progress[i] = 0.;
      /// Mod End
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

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
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
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
      if(progress[j] < _durations[j])
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

void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);

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

void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsBetweenMPC);// % _nIterations;
  for(int i = 0; i < 4; i++) {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float) (iterationsBetweenMPC * _periods[i]);
  }

  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);

}

int OffsetDurationGait::getCurrentGaitPhase() {
  return _iteration;
}

int MixedFrequncyGait::getCurrentGaitPhase() {
  return 0;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  return dtMPC * _swing;
}

float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) {
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;
  return dtMPC * _stance;
}

float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) {
  return dtMPC * _duty_cycle * _periods[leg];
}

void OffsetDurationGait::debugPrint() {

}

void MixedFrequncyGait::debugPrint() {

}