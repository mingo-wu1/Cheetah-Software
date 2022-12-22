# 四足机器人StandUp模式解析

StandUp模式下是四足机器人从原地趴着不动到BalanceStand的中间状态。

设定最高站立高度为0.25；

令进程$progress=0.5\cdot iter \cdot dt$；

设定四条腿的足端$K_{P},K_{D}$；

足端目标位置的$X,Y$保持切换时的初状态，足端高度逐渐逼近$-0.25$。逼近方法

```
this->_data->_legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
```

随着$progress$的增加，逐渐让足端与肩部的位置达到期望的高度值。代码中的pDes是足端相对于肩部的位置。（具体定义可以在ConvexMPCLocomotion.cpp中Line805明确）。



总结：StandUp模式是采用的位置环。