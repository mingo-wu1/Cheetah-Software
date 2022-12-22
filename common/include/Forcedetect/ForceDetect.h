/*!
 * @file    Forcedetect.h
 * @author  wangjie237, hanyuanqiang
 * @brief   Detect the extern force
 *
 * Use the equation of force balance:
 * A * qdd + b(q, qd) + g = tau + Jc.Transpose() * F_c
 */

#ifndef PROJECT_FOECEDETECT_H
#define PROJECT_FOECEDETECT_H

#include <iostream>
#include "cppTypes.h"
#include "Dynamics/Quadruped.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/LegController.h"
#include "Math/orientation_tools.h"
#include "Math/FirstOrderIIRFilter.h"
#include "Tools/Filters/AverageFilter.h"
#include "Tools/Filters/Polyfit.h"
#include "Tools/Filters/AmplitudeFilter.h"
#include <lcm-cpp.hpp>
#include "collision.hpp"
#include "Controllers/OrientationEstimator.h"

#define FILTER_NUM (18)

template <typename T>
class ForceDetect {    
 public:
    ForceDetect(LegController<T>* legcontroller, FloatingBaseModel<T>* model, StateEstimate<T>* stateEstimator, float dt);
    void run();
    /*!
     * 返回碰撞判定结果，4*1向量
     */
    DVec<bool> getContactState();
    /*!
     * 返回每个足端作用力，12*1向量
     */
    DVec<T> getContactForce();
    /*!
     * 返回每个关节力矩，12*1向量
     */
    DVec<T> getTau();
    /*!
     * 返回每关节角度，12*1向量
     */
    DVec<T> getQ();
    /*!
     * 返回四条腿承受的总重力
     */
    T getStaticTotalWeight();
    /*!
     * 返回负载重力，输入为本体质量
     */
    T getLoadWeight(T);
 private:
    LegController<T>* _legcontroller;
    FloatingBaseModel<T>* _model;
    StateEstimate<T>* _stateEstimator;
    T _dt;
    DMat<T> _A;
    DVec<T> _grav;
    DVec<T> _coriolis;
    DMat<T> _Jc18;
    DMat<T> _Jc18Tr_bottom12_inv;
    DVec<T> _Fc;
    DVec<T> _q;
    DVec<T> _qd;
    DVec<T> _qd_filtered;
    DVec<T> _qdd_filtered;
    DVec<T> _tau;
    FBModelState<T> _state;
    DVec<bool> _contact_judge;

    lcm::LCM _collisionLCM;
    collision collision_lcm;
    BaseFilter<T>* _filter[FILTER_NUM];
    AmplitudeFilter<T>* _amplitude_filter[4];

    void updateQdd();
    void updateParameters();
    void computeForceExt();
    void publishLCM();
};

#endif 