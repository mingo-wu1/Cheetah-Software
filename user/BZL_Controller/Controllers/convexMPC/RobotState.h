#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
// 【M模型范畴】单刚体动力学模型表示为连续状态空间方程
class RobotState // 机器人的状态表出（包括躯干惯量、位置、线速度、角速度、旋转角、偏航角、足端位置）
{
    public:
    
    	/// Mod Begin by yuzhiyou 2021-04-13, mod robot state
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, const flt* rpy); // 把机器人状态数组值转换成矩阵，方便计算连续状态空间方程
	/// Ori Code:
	// void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
	/// Mod End

        //void compute_rotations();
        void print(); // 打印机器人状态（包括躯干惯量、位置、线速度、角速度、旋转角、偏航角、足端位置）
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        /// Mod Begin by zhaoyudong peibo,2021-04-29,Modifying dynamic parameters
        /// Change by hanyuanqiang, 2021-08-10, Add unitree RS485 A1 motor parameters
#if (USE_RS485_A1 == 1)
        fpt m = 12.5;
#elif (USE_LINKAGE == 1)
        fpt m = 11.18;
#elif (USE_LINKAGE_INDUSTRIAL == 1)
        fpt m = 12; //14.48
#else
        fpt m = 10.6;
#endif
        /// Add Begin by yuzhiyou, 2021-04-13, mod robot state
        Matrix<fpt, 3, 3> R_rpy;
        /// Add End
        /// Ori Code:
        // fpt m = 9;
        /// Mod End;
        //fpt m = 50.236; //DH
    //private:
};
#endif
