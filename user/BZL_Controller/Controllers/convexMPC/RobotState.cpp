#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

// Mod Begin yuzhiyou 2021-04-13,Mod robot state
void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_, const flt* rpy_)
{
    for (u8 i = 0; i < 3; i++)
    {
	this->p(i) = p_[i];
	this->v(i) = v_[i];
	this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = rpy_[2];

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for (u8 rs = 0; rs < 3; rs++)
	for (u8 c = 0; c < 4; c++)
	    this->r_feet(rs, c) = r_[rs * 4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(this->yaw);
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

    Matrix<fpt,3,1> Id;
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

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



