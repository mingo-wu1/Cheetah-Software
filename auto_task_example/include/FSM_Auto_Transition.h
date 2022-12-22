/*!
 * @file    FSM_Auto_Transition.h
 * @author  peibo
 * @brief   Automatically complete the process of switching between different state machines
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <functional>
#include "FSM_State.h"

typedef union
{
	unsigned char total;
	struct
	{
		unsigned char nonempty : 1;           //This flag means whether the state machine can run.
		unsigned char passive : 1;            //This flag means that you can only switch directly to the state machine
		unsigned char transition : 6;         //This flag indicates the priority of the state machine as an intermediate transition
	};
}FSM_State_Flag;

typedef struct
{
	FSM_State_Flag flags;
	FSM_StateName stateName;
	std::vector<int> nextState;
	std::vector<int> preState;
}FSM_Infomation;


class FSM_Auto_Transition
{
public:
	FSM_Auto_Transition();
	FSM_StateName checkTransition(FSM_StateName src, int dst);
	FSM_StateName checkTransition(int src, int dst);
	void setUseSafeTransition(bool _flag) { _isUseSafeTransition = _flag; };

	int FSMStateToControlMode(FSM_StateName);
	FSM_StateName controlModeToFSMState(int);
	bool canTrans(int src, int dst);
	bool canTrans(int src, int mid, int dst);
private:
	void updateTransList(int src, int dst);
	void initControlModeToFSMState();
	void initTransition();
	void setNextState(std::map<int, FSM_Infomation> & source, int index, std::vector<int> _next);
	bool _isUseSafeTransition = true;
	int _safeTransitionState;
	int _lastControlMode = -1;

	std::map<int, FSM_StateName> _controlModeToFSMState;
	std::map<FSM_StateName, int> _FSMStateToControlMode;
	std::map<int, FSM_Infomation> _allState;
	std::vector<int> _allTrans;
	std::vector<int> _list;
};