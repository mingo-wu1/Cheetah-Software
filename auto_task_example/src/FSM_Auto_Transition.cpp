/*!
 * @file    FSM_Auto_Transition.cpp
 * @author  peibo
 * @brief   Automatically complete the process of switching between different state machines
 *
 */
#include "FSM_Auto_Transition.h"

using namespace std;

FSM_Auto_Transition::FSM_Auto_Transition()
{
	initControlModeToFSMState();
	initTransition();
}

FSM_StateName FSM_Auto_Transition::checkTransition(FSM_StateName src, int dst)
{
	int srcControl = FSMStateToControlMode(src);
	return checkTransition(srcControl, dst);
}

FSM_StateName FSM_Auto_Transition::checkTransition(int src, int dst)
{
	if (src == dst)
	{
		_lastControlMode = dst;
		return controlModeToFSMState(dst);
	}
	if (_lastControlMode != dst)
	{
		updateTransList(src, dst);
	}
	size_t index = 0;
	for (size_t i = 0; i < _list.size(); i++)
	{
		if (_list[i] == src)
		{
			index = (i + 1);
			index = index >= _list.size() ? index - _list.size() : index;
		}
	}
	return controlModeToFSMState(_list[index]);
}


void FSM_Auto_Transition::updateTransList(int src, int dst)
{
	bool transFlag = canTrans(src, dst);
	bool isUpdateList = false;
	if ((!transFlag || _isUseSafeTransition) && src != _safeTransitionState && dst != _safeTransitionState)
	{
		if (_allState.find(_safeTransitionState) != _allState.end())
		{
			if (canTrans(src, _safeTransitionState, dst))
			{
				_list.clear();
				_list.push_back(_safeTransitionState);
				_list.push_back(dst);
				isUpdateList = true;
				std::cout << "[CONTROL FSM] Now,transitionfrom list: "
					<< src << "," << _safeTransitionState << "," << dst << std::endl;
			}
		}
	}
	if (!isUpdateList && transFlag)
	{
		_list.clear();
		_list.push_back(dst);
		isUpdateList = true;
		std::cout << "[CONTROL FSM] Now,transitionfrom "
			<< src << " to " << dst << std::endl;
	}
	if (!isUpdateList)
	{
		_list.clear();
		for (auto tempMid : _allTrans)
		{
			if ((!_isUseSafeTransition || tempMid != _safeTransitionState) && canTrans(src, tempMid, dst))
			{
				_list.clear();
				_list.push_back(tempMid);
				_list.push_back(dst);
				isUpdateList = true;
				std::cout << "[CONTROL FSM] Now,transitionfrom list: "
					<< src << "," << tempMid << "," << dst << std::endl;
			}
		}
		if (_list.empty())
		{
			_list.push_back(src);
			std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
				<< src << " to " << dst << std::endl;
		}
	}
	_lastControlMode = dst;
}

void FSM_Auto_Transition::initControlModeToFSMState()
{
	_controlModeToFSMState.clear();
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_PASSIVE, FSM_StateName::PASSIVE));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_STAND_UP, FSM_StateName::STAND_UP));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_BALANCE_STAND, FSM_StateName::BALANCE_STAND));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_LOCOMOTION, FSM_StateName::LOCOMOTION));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_RECOVERY_STAND, FSM_StateName::RECOVERY_STAND));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_VISION, FSM_StateName::VISION));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_BACKFLIP, FSM_StateName::BACKFLIP));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_FRONTJUMP, FSM_StateName::FRONTJUMP));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_PRONE, FSM_StateName::PRONE));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_JOINT_PD, FSM_StateName::JOINT_PD));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_IMPEDANCE_CONTROL, FSM_StateName::IMPEDANCE_CONTROL));
	_controlModeToFSMState.insert(pair<int, FSM_StateName>(K_INVALID, FSM_StateName::INVALID));
	std::map<int, FSM_StateName>::iterator curIt;
	for (curIt = _controlModeToFSMState.begin(); curIt != _controlModeToFSMState.end(); curIt++)
	{
		_FSMStateToControlMode.insert(pair<FSM_StateName, int>(curIt->second, curIt->first));
	}
}

void FSM_Auto_Transition::initTransition()
{
	FSM_State_Flag tempFlag;
	tempFlag.total = 0;
	tempFlag.nonempty = 1;
	std::map<int, FSM_StateName>::iterator curIt;
	for (curIt = _controlModeToFSMState.begin(); curIt != _controlModeToFSMState.end(); curIt++)
	{
		_allState[curIt->first] = FSM_Infomation({ tempFlag,curIt->second, vector<int>(), vector<int>() });
	}
	_allState[K_INVALID].flags.total = 0;
	_allState[K_PASSIVE].flags.passive = 1;
	_allState[K_LOCOMOTION].flags.transition = 1;
	_allState[K_BALANCE_STAND].flags.transition = 10;
	_allState[K_RECOVERY_STAND].flags.transition = 60;

	_safeTransitionState = K_RECOVERY_STAND;

	_allTrans.push_back(K_RECOVERY_STAND);
	_allTrans.push_back(K_BALANCE_STAND);
	_allTrans.push_back(K_LOCOMOTION);

	setNextState(_allState, K_PASSIVE, vector<int>({ K_PASSIVE,K_STAND_UP,K_RECOVERY_STAND,K_JOINT_PD }));
	setNextState(_allState, K_STAND_UP, vector<int>({ K_PASSIVE,K_STAND_UP,K_LOCOMOTION,K_VISION }));
	setNextState(_allState, K_BALANCE_STAND, vector<int>({ K_PASSIVE,K_BALANCE_STAND,K_LOCOMOTION,K_RECOVERY_STAND,K_VISION,K_BACKFLIP }));
	setNextState(_allState, K_LOCOMOTION, vector<int>({ K_PASSIVE,K_STAND_UP,K_LOCOMOTION,K_RECOVERY_STAND,K_VISION }));
	setNextState(_allState, K_RECOVERY_STAND, vector<int>({ K_PASSIVE,K_BALANCE_STAND,K_LOCOMOTION,K_RECOVERY_STAND,K_VISION,K_BACKFLIP,K_FRONTJUMP,K_PRONE }));
	setNextState(_allState, K_VISION, vector<int>({ K_PASSIVE,K_BALANCE_STAND,K_LOCOMOTION,K_RECOVERY_STAND,K_VISION }));
	setNextState(_allState, K_BACKFLIP, vector<int>({ K_PASSIVE,K_BALANCE_STAND,K_LOCOMOTION,K_RECOVERY_STAND,K_BACKFLIP }));
	setNextState(_allState, K_FRONTJUMP, vector<int>({ K_PASSIVE,K_BALANCE_STAND,K_LOCOMOTION,K_RECOVERY_STAND,K_FRONTJUMP }));
	setNextState(_allState, K_PRONE, vector<int>({ K_PASSIVE,K_RECOVERY_STAND,K_PRONE }));
	setNextState(_allState, K_JOINT_PD, vector<int>({ K_PASSIVE,K_STAND_UP,K_BALANCE_STAND,K_JOINT_PD,K_IMPEDANCE_CONTROL }));
	setNextState(_allState, K_IMPEDANCE_CONTROL, vector<int>({ K_BALANCE_STAND,K_IMPEDANCE_CONTROL }));

}

void FSM_Auto_Transition::setNextState(map<int, FSM_Infomation>& source, int index, vector<int> _next)
{
	source[index].nextState = _next;
	for (auto tempA : _next)
	{
		source[tempA].preState.push_back(index);
	}
}

int FSM_Auto_Transition::FSMStateToControlMode(FSM_StateName stateName)
{
	auto iter = _FSMStateToControlMode.find(stateName);
	if (iter != _FSMStateToControlMode.end())
	{
		return iter->second;
	}
	else
	{
		return K_INVALID;
	}
}

FSM_StateName FSM_Auto_Transition::controlModeToFSMState(int controlMode)
{
	auto iter = _controlModeToFSMState.find(controlMode);
	if (iter != _controlModeToFSMState.end())
	{
		return iter->second;
	}
	else
	{
		return FSM_StateName::INVALID;
	}
}

bool FSM_Auto_Transition::canTrans(int src, int dst)
{
	vector<int>& tempNext = _allState[src].nextState;
	for (auto tmp : tempNext)
	{
		if (tmp == dst)
		{
			return true;
		}
	}
	return false;
}

bool FSM_Auto_Transition::canTrans(int src, int mid, int dst)
{
	if (_allState[dst].flags.passive || _allState[mid].flags.transition == 0)
	{
		return false;
	}
	if (!canTrans(src, mid) || !canTrans(mid, dst))
	{
		return false;
	}
	return true;
}
