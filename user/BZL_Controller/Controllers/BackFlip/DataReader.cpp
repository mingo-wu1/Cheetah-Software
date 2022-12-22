#include "DataReader.hpp"
#include <Configuration.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

DataReader::DataReader(const RobotType& type, FSM_StateName stateNameIn) : 
  _type(type), _logger("BackFlip DataReader")
{
  if (_type == RobotType::MINI_CHEETAH) {
    
    if (stateNameIn == FSM_StateName::BACKFLIP) {
      //load_control_plan(THIS_COM "user/WBC_Controller/WBC_States/BackFlip/data/mc_flip.dat");
      load_control_plan(THIS_COM "config/mc_flip.dat");
      QUADRUPED_INFO(_logger, "Backflip DataReader Setup for mini cheetah");
    }
    else if (stateNameIn == FSM_StateName::FRONTJUMP) {
      //load_control_plan(THIS_COM "user/BZL_Controller/Controllers/FrontJump/front_jump_data.dat"); // front_jump_data.dat for succesfull test 1 file
      load_control_plan(THIS_COM "config/front_jump_pitchup_v2.dat");
      QUADRUPED_INFO(_logger, "Front Jump DataReader Setup for mini cheetah");
    }
  } else {
    QUADRUPED_INFO(_logger, "Backflip DataReader Setup for mini cheetah 3");
    load_control_plan(THIS_COM "user/WBC_Controller/WBC_States/BackFlip/data/backflip.dat");
  }
}

void DataReader::load_control_plan(const char* filename) {
  QUADRUPED_INFO(_logger, "Loading control plan %s...", filename);
  FILE* f = fopen(filename, "rb");
  if (!f) {
    QUADRUPED_ERROR(_logger, "Error loading control plan!");
    return;
  }
  fseek(f, 0, SEEK_END);
  uint64_t file_size = ftell(f);
  fseek(f, 0, SEEK_SET);

  QUADRUPED_INFO(_logger, "Allocating %ld bytes for control plan", file_size);

  plan_buffer = (float*)malloc(file_size + 1);

  if (!plan_buffer) {
    QUADRUPED_ERROR(_logger, "malloc failed!");
    fclose(f);
    f = nullptr;
    return;
  }

  uint64_t read_success = fread(plan_buffer, file_size, 1, f);
  if (!read_success) {
    QUADRUPED_ERROR(_logger, "fread failed.");
  }

  if (file_size % sizeof(float)) {
    QUADRUPED_ERROR(_logger, "file size isn't divisible by size of float!");
  }

  fclose(f);

  plan_loaded = true;
  plan_timesteps = file_size / (sizeof(float) * plan_cols);
  QUADRUPED_INFO(_logger, "Done loading plan for %d timesteps", plan_timesteps);
}

float* DataReader::get_initial_configuration() {
  if (!plan_loaded) {
    QUADRUPED_ERROR(_logger, "get_initial_configuration called without a plan!");
    return nullptr;
  }

  return plan_buffer + 3;
}

float* DataReader::get_plan_at_time(int timestep) {
  if (!plan_loaded) {
    QUADRUPED_ERROR(_logger, "get_plan_at_time called without a plan!");
    return nullptr;
  }

  if (timestep < 0 || timestep >= plan_timesteps) {
    QUADRUPED_ERROR(_logger, 
      "get_plan_at_time called for timestep %d\n\tmust be between 0 and %d",
      timestep, plan_timesteps - 1);
    timestep = plan_timesteps - 1;
    // return nullptr; // TODO: this should estop the robot, can't really
    // recover from this!
  }

  // if(timestep < 0) { return plan_buffer + 3; }
  // if(timestep >= plan_timesteps){ timestep = plan_timesteps-1; }

  return plan_buffer + plan_cols * timestep;
}

void DataReader::unload_control_plan() {
  free(plan_buffer);
  plan_timesteps = -1;
  plan_loaded = false;
  QUADRUPED_INFO(_logger, "Unloaded plan.");
}
