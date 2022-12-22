#include "WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _wbcLCM(getLcmUrl(0))
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  // TEST
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data){
 
  /// Add Begin by wuchunming, 2021-03-10, add BZL parameters
  /*
  _wbic_data->_W_floating[0] = data.userParameters->w_floating1;
  _wbic_data->_W_floating[1] = data.userParameters->w_floating2;
  _wbic_data->_W_floating[2] = data.userParameters->w_floating3;
  _wbic_data->_W_floating[3] = data.userParameters->w_floating4;
  _wbic_data->_W_floating[4] = data.userParameters->w_floating5;
  _wbic_data->_W_floating[5] = data.userParameters->w_floating6;
  _wbic_data->_W_rf[0] = data.userParameters->w_rf1;
  _wbic_data->_W_rf[1] = data.userParameters->w_rf2;
  _wbic_data->_W_rf[2] = data.userParameters->w_rf3;
  _wbic_data->_W_rf[3] = data.userParameters->w_rf4;
  _wbic_data->_W_rf[4] = data.userParameters->w_rf5;
  _wbic_data->_W_rf[5] = data.userParameters->w_rf6;
  _wbic_data->_W_rf[6] = data.userParameters->w_rf7;
  _wbic_data->_W_rf[7] = data.userParameters->w_rf8;
  _wbic_data->_W_rf[8] = data.userParameters->w_rf9;
  _wbic_data->_W_rf[9] = data.userParameters->w_rf10;
  _wbic_data->_W_rf[10] = data.userParameters->w_rf11;
  _wbic_data->_W_rf[11] = data.userParameters->w_rf12;
  */
  /// Add End

  ++_iter;

  // Update Model
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Task & Contact Update
  _ContactTaskUpdate(input, data);

  // WBC Computation
  _ComputeWBC();
  
  // TEST
  //T dt(0.002);
  //for(size_t i(0); i<12; ++i){
    //_des_jpos[i] = _state.q[i] + _state.qd[i] * dt + 0.5 * _wbic_data->_qddot[i+6] * dt * dt;
    //_des_jvel[i] = _state.qd[i] + _wbic_data->_qddot[i+6]*dt;
  //}

  //_ContactTaskUpdateTEST(input, data);
  //_ComputeWBC();
  // END of TEST

  // Update Leg Command
  _UpdateLegCMD(data);

  // LCM publish
  _LCM_PublishData();
}



template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data){
  LegControllerCommand<T> * cmd = data._legController->commands;
  //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
	    cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];
    /// Mod begin by peibo, 2021-05-11,adding WBC operation separation in different modes
    if (leg == 0)
    {
      _Kp_joint[jidx] = data.userParameters->Kp_joint_fr_wbc[jidx];
      _Kd_joint[jidx] = data.userParameters->Kd_joint_fr_wbc[jidx];
    }
    else if (leg == 1)
    {
      _Kp_joint[jidx] = data.userParameters->Kp_joint_fl_wbc[jidx];
      _Kd_joint[jidx] = data.userParameters->Kd_joint_fl_wbc[jidx];
    }
    else if (leg == 2)
    {
      _Kp_joint[jidx] = data.userParameters->Kp_joint_br_wbc[jidx];
      _Kd_joint[jidx] = data.userParameters->Kd_joint_br_wbc[jidx];
    }
    else if (leg == 3)
    {
      _Kp_joint[jidx] = data.userParameters->Kp_joint_bl_wbc[jidx];
      _Kd_joint[jidx] = data.userParameters->Kd_joint_bl_wbc[jidx];
    }
   /// Ori Code:
      /// Mod begin by peibo 2021-03-29,automatically change kp,kd parameters with speed
      // if (leg == 0)
      // {
      //   switch ((int)data.userParameters->cur_speed_level)
      //   {
      //   case 1:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_fr_heighSpeed_1[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_fr_heighSpeed_1[jidx];
      //     break;
      //   case 2:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_fr_heighSpeed_2[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_fr_heighSpeed_2[jidx];
      //     break;
      //   case 0:
      //   default:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex1[jidx];
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex1[jidx]; 
      //     break;
      //   }
      // }
      // else if (leg == 1)
      // {
      //   switch ((int)data.userParameters->cur_speed_level)
      //   {
      //   case 1:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_fl_heighSpeed_1[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_fl_heighSpeed_1[jidx];
      //     break;
      //   case 2:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_fl_heighSpeed_2[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_fl_heighSpeed_2[jidx];
      //     break;
      //   case 0:
      //   default:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex2[jidx];
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex2[jidx];
      //     break;
      //   }
      // }
      // else if (leg == 2)
      // {
      //   switch ((int)data.userParameters->cur_speed_level)
      //   {
      //   case 1:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_br_heighSpeed_1[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_br_heighSpeed_1[jidx];
      //     break;
      //   case 2:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_br_heighSpeed_2[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_br_heighSpeed_2[jidx];
      //     break;
      //   case 0:
      //   default:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex3[jidx];
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex3[jidx];
      //     break;
      //   }
      // }
      // else if (leg == 3)
      // {
      //   switch ((int)data.userParameters->cur_speed_level)
      //   {
      //   case 1:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_bl_heighSpeed_1[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_bl_heighSpeed_1[jidx];
      //     break;
      //   case 2:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_bl_heighSpeed_2[jidx];
      //     _Kd_joint[jidx] = data.userParameters->Kd_joint_bl_heighSpeed_2[jidx];
      //     break;
      //   case 0:
      //   default:
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex4[jidx];
      //     _Kp_joint[jidx] = data.userParameters->Kp_joint_ex4[jidx];
      //     break;
      //   }
      // }
    /// Mod End
      cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx]; 
      /// Ｏrigin Ｃode:

      /// Mod Begin by wuchunming, 2021-03-10, add BZL parameter
      /*
      if(0 == leg){
        cmd[leg].kpJoint(jidx, jidx) = data.userParameters->Kp_joint_ex1[jidx];
        cmd[leg].kdJoint(jidx, jidx) = data.userParameters->Kd_joint_ex1[jidx];
      }
      else if(1 == leg){
        cmd[leg].kpJoint(jidx, jidx) = data.userParameters->Kp_joint_ex2[jidx];
        cmd[leg].kdJoint(jidx, jidx) = data.userParameters->Kd_joint_ex2[jidx];
      }
      else if(2 == leg){
        cmd[leg].kpJoint(jidx, jidx) = data.userParameters->Kp_joint_ex3[jidx];
        cmd[leg].kdJoint(jidx, jidx) = data.userParameters->Kd_joint_ex3[jidx];
      }
      else if(3 == leg){
        cmd[leg].kpJoint(jidx, jidx) = data.userParameters->Kp_joint_ex4[jidx];
        cmd[leg].kdJoint(jidx, jidx) = data.userParameters->Kd_joint_ex4[jidx];
      }
      */
      /// origin code
      // cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      // cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
      /// Mod End

       //if(contact[leg] > 0.){ // Contact
        //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
        //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
      //}else{
        //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint_swing[jidx];
        //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint_swing[jidx];
      //}
      /// Mod End
    }
  }

// TODO(hanyuanqiang): Single step test, to be deleted in the future
#if 0
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) 
  {
    if ((leg != 0) && (leg != 1))
    {
      cmd[leg].zero();
      Vec3<float> stand_jpos;
      stand_jpos << 0.f, -.8f, 1.6f;

      for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
        cmd[leg].tauFeedForward[jidx] = 0;
        cmd[leg].qDes[jidx] = stand_jpos[jidx];
        cmd[leg].qdDes[jidx] = 0;
        cmd[leg].kpJoint(jidx, jidx) = 0.025;//_Kp_joint_all[leg][jidx];
        cmd[leg].kdJoint(jidx, jidx) = 0.05;//_Kd_joint_all[leg][jidx];
      }

    }
  }
#endif
/// Add End

  // Knee joint non flip barrier
  for(size_t leg(0); leg<4; ++leg){
    if(cmd[leg].qDes[2] < 0.3){
      cmd[leg].qDes[2] = 0.3;
    }
    if(data._legController->datas[leg].q[2] < 0.3){
      T knee_pos = data._legController->datas[leg].q[2]; 
      cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
    }
  }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    const LegControllerData<T> * leg_data){

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
