/*!
 * @file    Forcedetect.cpp
 * @author  wangjie237, hanyuanqiang
 * @brief   Detect the extern force
 *
 * Use the equation of force balance:
 * A * qdd + b(q, qd) + g = tau + Jc.Transpose() * F_c
 */
#include "Forcedetect/ForceDetect.h"

template <typename T>
ForceDetect<T>::ForceDetect(LegController<T>* legcontroller, FloatingBaseModel<T>* model, StateEstimate<T>* stateEstimator, float dt):
    _legcontroller(legcontroller),
	_model(model),
	_stateEstimator(stateEstimator),
 	_dt(dt),
    _collisionLCM(getLcmUrl(0)) {
 	_A = DMat<T>(18, 18);
    _grav = DVec<T>::Zero(18);
    _coriolis = DVec<T>::Zero(18);
    _Jc18 = DMat<T>(12, 18);
    _Jc18Tr_bottom12_inv = DMat<T>(12, 12);
    _Fc = DVec<T>::Zero(12);
    _q = DVec<T>::Zero(18);
    _qd = DVec<T>::Zero(18);
    _qd_filtered = DVec<T>::Zero(18);
    _qdd_filtered = DVec<T>::Zero(18);
    _tau = DVec<T>::Zero(12);
    _state.q = DVec<T>::Zero(18);
    _state.qd = DVec<T>::Zero(18);
    _contact_judge = DVec<bool>::Zero(4);

    for (int i = 0; i < FILTER_NUM; i++)
    {
        _filter[i] = new AverageFilter<T>(4);
    }

    for (int i = 0; i < 4; i++)
    {
        _amplitude_filter[i] = new AmplitudeFilter<T>(5, 20); 
    }
}

template <typename T>
void ForceDetect<T>::updateQdd(){
    _qd.segment(0,3) = _stateEstimator->omegaBody;
    _qd.segment(3,3) = _stateEstimator->vBody;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            _qd[3*i + j + 6] = _legcontroller->datas[i].qd(j);
        }
    }

    for (int i = 0; i < FILTER_NUM; i++)
    {
        _filter[i]->update(_qd[i]);
        _qdd_filtered[i] = _filter[i]->getFilteredDataDeriviation() / _dt;
    }
}

template <typename T>
void ForceDetect<T>::updateParameters(){
    updateQdd();
    _q.segment(0,3) = _stateEstimator->rpy;
    _q.segment(3,3) = _stateEstimator->position;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            _q[3*i + j + 6] = _legcontroller->datas[i].q(j);
#if (USE_SPI_DATA_CURRENT == 1)
            _tau[3*i + j] = _legcontroller->datas[i].tauMeasure(j);
#else
            _tau[3*i + j] = _legcontroller->datas[i].tauEstimate(j);
#endif
        }
    }
    
    _state.bodyOrientation = _stateEstimator->orientation;
    _state.bodyPosition = _stateEstimator->position;
    for(size_t i(0); i<3; ++i){
        _state.bodyVelocity[i] = _stateEstimator->omegaBody[i];
        _state.bodyVelocity[i+3] = _stateEstimator->vBody[i];
        for(size_t j(0); j<4; ++j){
            _state.q[3*j + i] = _legcontroller->datas[j].q(i);
            _state.qd[3*j + i] = _legcontroller->datas[j].qd(i);
        }
    }

    _model->setState(_state);

    _model->contactJacobians();
    _model->massMatrix();
    _model->generalizedGravityForce();
    _model->generalizedCoriolisForce();

    _A = _model->getMassMatrix();
    _grav = _model->getGravityForce();
    _coriolis = _model->getCoriolisForce();
    _Jc18.block(0,0,3,18) = _model->_Jc[9];
    _Jc18.block(3,0,3,18) = _model->_Jc[11];
    _Jc18.block(6,0,3,18) = _model->_Jc[13];
    _Jc18.block(9,0,3,18) = _model->_Jc[15];

    _Jc18Tr_bottom12_inv = ((_Jc18.transpose()).block(6, 0, 12, 12)).inverse();
}

template <typename T>
void ForceDetect<T>::computeForceExt(){
    _Fc = _Jc18Tr_bottom12_inv * (_A.block(6,0,12,18) * _qdd_filtered + _coriolis.tail(12) + _grav.tail(12) - _tau);
    for (int i = 0; i < 4; i++){
        _amplitude_filter[i]->update(_Fc[3*i + 2]);
        _contact_judge[i] = (bool)_amplitude_filter[i]->getFilteredData();
    }
}

template <typename T>
void ForceDetect<T>::run(){
    updateParameters();
    computeForceExt();
    publishLCM();
}

template <typename T>
DVec<bool> ForceDetect<T>::getContactState(){
    return _contact_judge;
}

template <typename T>
DVec<T> ForceDetect<T>::getContactForce(){
    return _Fc;
}

template <typename T>
DVec<T> ForceDetect<T>::getTau(){
    return _tau;
}

template <typename T>
DVec<T> ForceDetect<T>::getQ(){
    return _q;
}

template <typename T>
T ForceDetect<T>::getStaticTotalWeight(){
    return _Fc[2] + _Fc[5] + _Fc[8] + _Fc[11];
}

template <typename T>
T ForceDetect<T>::getLoadWeight(T dogmass){
    return _Fc[2] + _Fc[5] + _Fc[8] + _Fc[11] - dogmass;
}

template <typename T>
void ForceDetect<T>::publishLCM(){
    for( int i = 0; i < 12; i++ )
  {
    collision_lcm.fc[i] = _Fc[i];
    collision_lcm.q[i] = _q(i);
    collision_lcm.qd[i] = _qd[i+6];
    collision_lcm.qdd[i] = _qdd_filtered[i+6];
    collision_lcm.b[i] = _coriolis[i+6];
    collision_lcm.g[i] = _grav[i+6];
    collision_lcm.tau[i] = _tau[i];
  }

  for (int i = 0; i < 4; i++){
      collision_lcm.contact_judge[i] = static_cast<double>(_contact_judge[i]);
  }
  
  _collisionLCM.publish("collision_test", &collision_lcm);
}

template class ForceDetect<double>;
template class ForceDetect<float>;
