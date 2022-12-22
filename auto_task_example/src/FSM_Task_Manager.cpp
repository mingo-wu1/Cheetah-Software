/*!
 * @file    FSM_Task_Manager.cpp
 * @author  peibo
 * @brief   Responsible for the execution of automatic tasks and the management of switching between state machines
 *
 */
#include "FSM_Task_Manager.h"

template<typename T>
inline FSM_StateName FSM_Task_Manager<T>::run(FSM_StateName src, double* dst_it)
{
	int dst = (int)(*dst_it);
	if (_cycle_index != 0 && !_taskList.empty())
	{
		if (_last_Task_index != _cur_Task_index)
		{
			_taskList[_cur_Task_index]->taskInit();
			_last_Task_index = _cur_Task_index;
			std::cout << _taskList[_cur_Task_index]->getName() << "  task start!" << std::endl;
		}
		int dstControlMode = FSMStateToControlMode(_taskList[_cur_Task_index]->getFSM_StateName());
		if (_taskList[_cur_Task_index]->checkFSMState(src) && _taskList[_cur_Task_index]->taskRun())
		{
			_taskList[_cur_Task_index]->taskExit();
			std::cout << _taskList[_cur_Task_index]->getName() << "  task finsh!" << std::endl;
			long int taskAllNum = _taskList.size();
			if (++_cur_Task_index >= taskAllNum)
			{
				_cur_Task_index = 0;
				_last_Task_index = -1;
				if (--_cycle_index == 0)
				{
					*dst_it = (double)dstControlMode;
					std::cout << "The circular task has ended!" << std::endl;
				}
				else
				{
					std::cout << "It's still ";
					if (_cycle_index < 0)
						std::cout << "Countless";
					else
						std::cout << _cycle_index;
					std::cout << " times to finish" << std::endl;
				}
			}
		}
		isUseGamepad = false;
		return checkTransition(src, dstControlMode);
	}
	else
	{
		isUseGamepad = true;
		return checkTransition(src, dst);
	}
}


template<typename T>
inline void FSM_Task_Manager<T>::addTask(FSM_Task<T>* task)
{
	if (task != NULL)
	{
		_taskList.push_back(task);
	}
}

template<typename T>
inline bool FSM_Task_Manager<T>::deleteTask(std::string& name)
{
	bool eraseFlag = false;
	for (size_t i = 0; i < _taskList.size(); i++)
	{
		if (_taskList[i]->getName() == name)
		{
			_taskList.erase(_taskList.begin() + i--);
			eraseFlag = true;
		}
	}
	if (eraseFlag)
	{
		return true;
	}
	std::cout << name << "  task does not exist!" << std::endl;
	return false;
}

template<typename T>
bool FSM_Task_Manager<T>::swapTask(std::string & name, unsigned long int index)
{
	if (index >= _taskList.size())
	{
		std::cout << "Array out of bounds!" << std::endl;
		return false;
	}
	for (size_t i = 0; i < _taskList.size(); i++)
	{
		if (_taskList[i]->getName() == name)
		{
			FSM_Task<T> * tmpa = _taskList[i];
			_taskList[i] = _taskList[index];
			_taskList[index] = tmpa;
		}
	}
	std::cout << name << "  task does not exist!" << std::endl;
	return false;
}


template class FSM_Task_Manager<float>;
template class FSM_Task_Manager<double>;