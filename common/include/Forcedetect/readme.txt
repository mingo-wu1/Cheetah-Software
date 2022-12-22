1. 将Forcedetect文件夹放在Cheetah/common/include下
2. 将Forcedetect下的collision_lcm文件放在Cheetah/lcm-types下
3. 命令行进入Cheetah/scripts，./make_types.sh生成对应lcm的文件
4. 在MIT_Controller.hpp中#include "Forcedetect/ForceDetect.h"，并在类声明的protected中声明变量ForceDetect<float>* _forceDetect;
5. 在MIT_Controller.cpp的initializeController()函数中，定义_forceDetect = new ForceDetect<float>(_legController, _model, _stateEstimate, _controlParameters->controller_dt);
6. 在MIT_Controller.cpp的runController()函数中，_forceDetect->run()进行足端碰撞检测

其余函数说明：
DVec<bool> getContactState();
返回碰撞判定结果，4*1向量
DVec<T> getContactForce();
返回每个足端作用力，12*1向量
DVec<T> getTau();
返回每个关节力矩，12*1向量
DVec<T> getQ();
返回每关节角度，12*1向量
T getStaticTotalWeight();
返回四条腿承受的总重力
T getLoadWeight(T);
返回负载重力，输入为本体质量