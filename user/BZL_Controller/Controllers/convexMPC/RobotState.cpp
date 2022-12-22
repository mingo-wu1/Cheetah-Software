#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

/**
 * 功能：把数组值转换成矩阵，方便计算连续状态空间方程
 *      机器人的状态表出（包括躯干惯量、位置、线速度、角速度、旋转角、偏航角、足端位置）
 * （1）p_   ：位置
 * （2）v_   ：速度
 * （3）q_   ：角速度
 * （4）w_   ：四元数
 * （5）r_   ：四脚从COM指向足端向量
 * （6）yaw_ ：偏航角
 */
// Mod Begin yuzhiyou 2021-04-13,Mod robot state
void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_, const flt* rpy_)
{
    for (u8 i = 0; i < 3; i++)
    {
	this->p(i) = p_[i]; // 位置
	this->v(i) = v_[i]; // 速度
	this->w(i) = w_[i]; // 角速度
    }
    this->q.w() = q_[0]; // 四元素
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = rpy_[2]; // 偏航角

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for (u8 rs = 0; rs < 3; rs++) // 四脚从COM指向足端向量
	for (u8 c = 0; c < 4; c++)
	    this->r_feet(rs, c) = r_[rs * 4 + c];

    R = this->q.toRotationMatrix(); // 机体旋转局矩阵
    fpt yc = cos(this->yaw); // Rz(fai)
    fpt ys = sin(this->yaw);

    R_yaw << yc, -ys, 0,
	     ys, yc, 0,
	      0, 0, 1;

    fpt pc = cos(rpy_[1]);
    fpt ptan = tan(rpy_[1]);
    if (fabs(pc) > 1e-6 )
    {
	R_rpy<< yc / pc,  -ys, yc*ptan,
		ys / pc,  yc, yc*ptan,
		      0,         0,  1;
    }
    else
    {
	R_rpy << yc, -ys, 0,
	         ys,  yc, 0,
	          0,   0,  1;
    }

    Matrix<fpt,3,1> Id; // 机身坐标系下惯性矩阵
    /// Mod Begin by zhaoyudong peibo,2021-04-29,Modifying dynamic parameters
    /// Change by hanyuanqiang, 2021-08-10, Add unitree RS485 A1 motor parameters
#if (USE_RS485_A1 == 1)
    Id << .0972f, 0.3611f, 0.33611f;
#elif (USE_LINKAGE == 1)
    Id << .0819f, 0.3042f, 0.28314f;
#elif (USE_LINKAGE_INDUSTRIAL == 1)
    Id << .0819f, 0.3042f, 0.28314f;
#else
    Id << .0819f, 0.3042f, 0.28314f;
#endif
    /// Ori Code:
    // Id << .07f, 0.26f, 0.242f;
    /// Mod End;
    //Id << 0.3f, 2.1f, 2.1f; // DH
    I_body.diagonal() = Id;

    //TODO: Consider normalizing quaternion??
}

// void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
// {
//     for(u8 i = 0; i < 3; i++)
//     {
//         this->p(i) = p_[i];
//         this->v(i) = v_[i];
//         this->w(i) = w_[i];
//     }
//     this->q.w() = q_[0];
//     this->q.x() = q_[1];
//     this->q.y() = q_[2];
//     this->q.z() = q_[3];
//     this->yaw = yaw_;

//     //for(u8 i = 0; i < 12; i++)
//     //    this->r_feet(i) = r[i];
//     for(u8 rs = 0; rs < 3; rs++)
//         for(u8 c = 0; c < 4; c++)
//             this->r_feet(rs,c) = r_[rs*4 + c];

//     R = this->q.toRotationMatrix();
//     fpt yc = cos(yaw_);
//     fpt ys = sin(yaw_);

//     R_yaw <<  yc,  -ys,   0,
//              ys,  yc,   0,
//                0,   0,   1;

//     Matrix<fpt,3,1> Id;
//     /// Mod Begin by zhaoyudong peibo,2021-04-29,Modifying dynamic parameters
//     Id << .0819f, 0.3042f, 0.28314f;
//     /// Ori Code:
//     // Id << .07f, 0.26f, 0.242f;
//     /// Mod End;
//     //Id << 0.3f, 2.1f, 2.1f; // DH
//     I_body.diagonal() = Id;

//     //TODO: Consider normalizing quaternion??
// }
/// Mod End

// 功能：机器人状态打印，包括位置、线速度、角速度、旋转角、偏航角、足端位置、身体惯量
void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



