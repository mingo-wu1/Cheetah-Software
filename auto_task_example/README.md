# 1. FSM_Task_Manager使用说明

## 1.1. 自定义任务

通过继承FSM_Task类进行编写处理功能

- **FSM_Task接口定义**

```c++
class FSM_Task
{
public:
    FSM_Task(ControlFSMData<T>* _FSMData, FSM_StateName stateName, std::string& name) :
        _controlFSMData(_FSMData), _FSM_StateName(stateName), _name(name){};
    virtual void taskInit() = 0;  //任务初始化
    virtual bool taskRun() = 0;   //任务执行函数，返回true表示结束
    virtual void taskExit() = 0;  //任务退出处理函数
    std::string getName() { return _name; };
    FSM_StateName getFSM_StateName() { return _FSM_StateName; };
    bool checkFSMState(FSM_StateName src) { return (_FSM_StateName == src); };
protected:
    ControlFSMData<T>* _controlFSMData;
private:
    FSM_StateName _FSM_StateName;      //该任务对应的状态机
    std::string _name;                 //任务的名称
};
```
## 1.2. 添加任务

添加过程可以参考以下代码
```c++
FSM_Task_Manager<T> _FSM_Task_Manager;
static FSM_Move_Task<float> tempMoveTask(0.2,0,0,0.002, 2, &data, "move_forward_0.2");
static FSM_Prone_Task<float> tempProneTask(0.002,5,&data,"Prone");
_FSM_Task_Manager.addTask(&tempMoveTask);
_FSM_Task_Manager.addTask(&tempProneTask);
```

## 1.3. 设置循环次数

通过使用FSM_Task_Manager中的setCycleIndex函数进行设定，cycle_index输入为正数时表示循环几次，0的时候为停止，负数表示无数次
```c++
void setCycleIndex(long int cycle_index)
```

# 2. FSM_Task_Manager添加到ControlFSM的示例

## 2.1. 修改ControlFSM头文件

在ControlFSM.h添加以下头文件和变量

```c++
#include "FSM_Task_Manager.h"

FSM_Task_Manager<T> _FSM_Task_Manager;
bool isUseFSMTaskManager = true;
```

## 2.2. 修改ControlFSM源文件示例

在ControlFSM.cpp中ControlFSM构造函数的结尾添加任务和循环次数

```c++
  static FSM_Move_Task<float> tempMoveTask(0.2,0,0,0.002, 2, &data, "move_forward_0.2");
  static FSM_Locomotion_Task<float> tempLocomotionTask(1,0.2,0,0,0.002, 2, &data, "move_forward_0.2");
  static FSM_Dance_Task<float> tempDanceTask(1,&data,"Dance");
  static FSM_BackFlip_Task<float> tempBackflipTask(&data,"Backflip");
  static FSM_Prone_Task<float> tempProneTask(0.002,3,&data,"Prone");
  
  _FSM_Task_Manager.addTask(&tempMoveTask);
  _FSM_Task_Manager.addTask(&tempLocomotionTask);
  _FSM_Task_Manager.addTask(&tempDanceTask);
  _FSM_Task_Manager.addTask(&tempBackflipTask);
  _FSM_Task_Manager.addTask(&tempProneTask);
  _FSM_Task_Manager.setCycleIndex(1); //The parameter entered is to set the number of cycles
```

在ControlFSM类中runFSM实现函数中，将原来的checkTransition替换为FSM_Task_Manager的run函数。

原来的部分：
```c++
nextStateName = currentState->checkTransition();
```

修改后：
```c++
if(isUseFSMTaskManager)
{
  nextStateName = _FSM_Task_Manager.run(currentState->stateName,&data.controlParameters->control_mode);
  currentState->nextStateName = nextStateName;
}
else
{
  nextStateName = currentState->checkTransition();
}
```