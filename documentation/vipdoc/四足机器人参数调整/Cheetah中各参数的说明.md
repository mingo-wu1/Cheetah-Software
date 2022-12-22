# Cheetah中各参数的说明

**RobotRunner.cpp,Line137-138**

设置默认的$kpJoint$和$kdJoint$

**JPosInitializer.cpp,Line46**

设置启动站立时的**位置环**

**俯仰角补偿，ConvexMPCLocomotion.cpp,Line216**
$$
\phi^{ini} = \phi^{ini} + dt \frac{\phi^{des} - \phi^{real}}{v_{x}}
$$
**滚转角补偿，ConvexMPCLocomotion.cpp,Line223**
$$
\theta^{ini} = \theta^{ini} + dt \frac{\theta^{des} - \theta^{real}}{v_{x}}
$$
同时$-0.25<\phi^{ini},\theta^{ini}<0.25$。

最终经补偿后的期望俯仰角和滚转角为：
$$
\begin{array}{l}
\phi^{comp} &= v_{x} * \phi^{ini} \\
\theta^{comp} &= v_{y} * \theta^{int}
\end{array}
$$
通过这样的方法完成俯仰角和滚转角的平滑。

**ConvexMPCLocomotion.cpp,Line467**

运动模式下$user\_wbc==0$时摆动设定的腿部控制设计，此时有$commands.pDes,commands.vDes,commands.kpCartesian,commands.kdCartesian$

**ConvexMPCLocomotion.cpp,Line493**

运动模式下$user\_wbc==0$时触地的腿部控制设计，此时有

$commands.pDes,commands.vDes,commands.kpCartesian,commands.kdCartesian,commands.forceFeedForward,commands.kdJoint$

**ConvexMPCLocomotion.cpp,Line508**

运动模式下$user\_wbc!=0$时触地的腿部，此时有

$commands.pDes,commands.vDes,commands.kpCartesian=0,commands.kdCartesian$

**FSM_State_Locomotion.cpp,Line288**

如果$user\_wbc<=0.9$的时候，所有参数不发生变化。

**FSM_State_Locomotion.cpp,Line268, WBC_Ctrl.cpp,Line113**

如果$user\_wbc>0.9$的时候，使用$WBC$，此时有

$commands.qDes,commands.qdDes,commands.tauFeedForward,commands.kpJoint,commands.kdJoint$

