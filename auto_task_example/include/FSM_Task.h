/*!
 * @file    FSM_Task.h
 * @author  peibo
 * @brief   Provides interface classes for automatic tasks and examples of some tasks
 *
 */

#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <functional>
#include "FSM_State.h"
#include "ControlFSMData.h"
#include <string>

template <typename T>
class FSM_Task
{
public:
    FSM_Task(ControlFSMData<T>* _FSMData, FSM_StateName stateName, std::string& name) :
        _controlFSMData(_FSMData), _FSM_StateName(stateName), _name(name){};
    virtual void taskInit() = 0;
    virtual bool taskRun() = 0;
    virtual void taskExit() = 0;
    std::string getName() { return _name; };
    FSM_StateName getFSM_StateName() { return _FSM_StateName; };
    bool checkFSMState(FSM_StateName src) { return (_FSM_StateName == src); };
protected:
    ControlFSMData<T>* _controlFSMData;

private:
    FSM_StateName _FSM_StateName;
    std::string _name;
};

template <typename T>
class FSM_Move_Task :public FSM_Task<T>
{
public:
    FSM_Move_Task(T forwardSpeed, T leftSpeed, T rotSpeed, T dt, T time,ControlFSMData<T>* _FSMData,std::string name) :
        FSM_Task<T>(_FSMData, FSM_StateName::LOCOMOTION, name),_forwardSpeed(forwardSpeed), _leftSpeed(leftSpeed),
        _rotSpeed(rotSpeed), _dt(dt), _time(time){};
    void taskInit()
    { 
        _remainingIter = (long int)(_time / _dt); 
        std::cout<<"Remaining iter : "<< _remainingIter <<std::endl;
    };
    virtual bool taskRun() 
    {
        this->_controlFSMData->_desiredStateCommand->rightAnalogStick[0] = _rotSpeed;
        this->_controlFSMData->_desiredStateCommand->leftAnalogStick[1] = _forwardSpeed;
        this->_controlFSMData->_desiredStateCommand->leftAnalogStick[0] = _leftSpeed;
        if (--_remainingIter <= 0)
        {
            return true;
        }
        return false;
    };
    void taskExit() {};

protected:
    T _forwardSpeed = 0, _leftSpeed = 0, _rotSpeed = 0, _dt = 0, _time = 0;
    long int _remainingIter = 0;

};

template <typename T>
class FSM_Dance_Task :public FSM_Task<T>
{
public:
    FSM_Dance_Task(int inputNum,ControlFSMData<T>* _FSMData,std::string name) :
        FSM_Task<T>(_FSMData,FSM_StateName::BALANCE_STAND , name),_inputNum(inputNum){};
    void taskInit()
    {
        this->_controlFSMData->controlParameters->use_cubicspline_for_qpstand_test = 1;
        this->_controlFSMData->autoTaskParam.balanceStandDanceNum = _inputNum;
    };
    bool taskRun() { return (this->_controlFSMData->controlParameters->use_cubicspline_for_qpstand_test == 0); };
    void taskExit(){};    
private:
    int _inputNum = 1;
};


template <typename T>
class FSM_Prone_Task :public FSM_Task<T>
{
public:
    FSM_Prone_Task(T dt, T time, ControlFSMData<T>* _FSMData, std::string name) :
        FSM_Task<T>(_FSMData, FSM_StateName::PRONE, name), _dt(dt), _time(time) {};
    void taskInit()
    {
        _remainingIter = (long int)(_time / _dt);
        std::cout << "Remaining iter : " << _remainingIter << std::endl;
    };
    bool taskRun() 
    {
        if (--_remainingIter <= 0)
        {
            return true;
        }
        return false;
    };
    void taskExit() {};

private:
    T _dt = 0, _time = 0;
    long int _remainingIter = 0;
};

/// Add Begin by peibo, 2020-09-06, add back flip task
template <typename T>
class FSM_BackFlip_Task :public FSM_Task<T>
{
public:
    FSM_BackFlip_Task(ControlFSMData<T>* _FSMData,std::string name) :
        FSM_Task<T>(_FSMData,FSM_StateName::BACKFLIP , name){};
    void taskInit()
    {
        this->_controlFSMData->autoTaskParam.backflipAutoTaskFlag = true;
    };
    bool taskRun() { return (this->_controlFSMData->autoTaskParam.backflipAutoTaskFlag == false); };
    void taskExit(){};    
};
/// Add End

/// Add Begin by peibo, 2020-09-06, add switch gait movement task
template <typename T>
class FSM_Locomotion_Task :public FSM_Move_Task<T>
{
public:
    FSM_Locomotion_Task(int gaitNum,T forwardSpeed, T leftSpeed, T rotSpeed, T dt, T time,ControlFSMData<T>* _FSMData,std::string name) :
        FSM_Move_Task<T>(forwardSpeed, leftSpeed, rotSpeed, dt, time, _FSMData, name),_gaitNum(gaitNum){};
    bool taskRun() 
    {
        this->_controlFSMData->_desiredStateCommand->rcCommand->variable[0] = _gaitNum;
        this->_controlFSMData->_desiredStateCommand->rightAnalogStick[0] = this->_rotSpeed;
        this->_controlFSMData->_desiredStateCommand->leftAnalogStick[1] = this->_forwardSpeed;
        this->_controlFSMData->_desiredStateCommand->leftAnalogStick[0] = this->_leftSpeed;
        if (--this->_remainingIter <= 0)
        {
            return true;
        }
        return false;
    };
    void taskExit() {};
private:
    int _gaitNum = 9;
};
/// Add End

/// Add Begin by peibo, 2020-09-06, add stand task
template <typename T>
class FSM_Stand_Task :public FSM_Task<T>
{
public:
	FSM_Stand_Task(T dt, T time, ControlFSMData<T>* _FSMData, std::string name) :
		FSM_Task<T>(_FSMData, FSM_StateName::RECOVERY_STAND, name), _dt(dt), _time(time) {};
	void taskInit()
	{
		_remainingIter = (long int)(_time / _dt);
		std::cout << "Remaining iter : " << _remainingIter << std::endl;
	};
	bool taskRun()
	{
		if (--_remainingIter <= 0)
		{
			return true;
		}
		return false;
	};
	void taskExit() {};

private:
	T _dt = 0, _time = 0;
	long int _remainingIter = 0;
};
/// Add End




template class FSM_Task<float>;
template class FSM_Move_Task<float>;
// template class FSM_Dance_Task<float>;


