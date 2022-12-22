#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
    
    	/// Mod Begin by yuzhiyou 2021-04-13, mod robot state
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, const flt* rpy);
	/// Ori Code:
	// void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
	/// Mod End

        //void compute_rotations();
        void print();
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
