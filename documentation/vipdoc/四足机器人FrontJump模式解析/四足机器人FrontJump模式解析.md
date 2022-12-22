# 四足机器人FrontJump模式解析

切换状态时定义了几个迭代计数器，$iter,\_state\_iter,\_count,\_motion_start_iter$；

记录了初始的关节角度；

设计关节的$K_{p},K_{D},K_{Pjoint},K_{DJoint}$;

$\_b\_running$始终为$true$；

进入前向跳跃初始化

1. 前6个周期，关节角设为初始值；
2. 每个关节的前向力矩为0、角加速度为0、设定为$K_{Pjoint},K_{DJoint}$;

初始化之后计算指令(ComputeCommand())；

初次进入控制计算时，记录当前时间；

进入$OneStep()$；

1. 状态机时间从控制计算开始；
2. 更新关节控制指令；
3. 将关节指令发送到关节控制器；
4. 如果达到中止相位，则进入$LastVisit()$,(实际上没干啥)；



其中关节指令计算