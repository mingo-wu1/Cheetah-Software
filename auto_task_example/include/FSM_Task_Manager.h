/*!
 * @file    FSM_Task_Manager.h
 * @author  peibo
 * @brief   Responsible for the execution of automatic tasks and the management of switching between state machines
 *
 */
#pragma once

#include "FSM_Auto_Transition.h"
#include "ControlFSMData.h"
#include <string>
#include "FSM_Task.h"

template <typename T>
class FSM_Task_Manager : public FSM_Auto_Transition
{
public:
    FSM_StateName run(FSM_StateName src, double* dst_it);
    void addTask(FSM_Task<T>*);
    bool deleteTask(std::string& name);
    bool swapTask(std::string& name,unsigned long int index);
    void setCycleIndex(long int cycle_index) { _cycle_index = cycle_index; }; //The parameter entered is to set the number of cycles,0 is stop, negative number means infinite times
    long int getCycleIndex() { return _cycle_index; };
    size_t getTaskNum() { return _taskList.size(); };
    bool getUseGamepad() {return isUseGamepad;};


private:
    bool isUseAutoTasks = true;
    std::vector<FSM_Task<T> * > _taskList;
    long int _cur_Task_index = 0;
    long int _last_Task_index = -1;
    long int _cycle_index = 0;
    bool isUseGamepad = true;

};
