/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "Math/Interpolation.h"
#include "Controllers/FootSwingTrajectory.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  T zp, zv, za;

  if(phase < T(0.5)) {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

/// Add Begin yuzhiyou, peibo, 2021-04-23, add select for selecting foot trajectory plan, polynomial algorithm
/*!
 * @brief foot trajectory plan, polynomial algorithm
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezierForStair(T phase, T swingTime) {
	_p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
	_v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
	_a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);
	Vec3<T> pva;
	Eigen::Matrix<T, 5, 1> Pos;
	Eigen::Matrix<T, 5, 1> Time;
	Pos << _p0[2], _p0[2] + _height * 0.75, _p0[2] + _height , _p0[2] + _height * 0.75, _pf[2];
	Time << 0, 0.25, 0.5, 0.75, 1;
	computeSwingTrajectoryPolynomial(phase, swingTime, Pos, Time, pva);
	_p[2] = pva(0);
	_v[2] = pva(1);
	_a[2] = pva(2);
	//x
	T sign = 1;
	if (_pf[0] < _p0[0])
	{
		sign = -1;
	}
	Pos << _p0[0], _p0[0] - sign * 0.01, (_p0[0] + _pf[0]) / 2, _pf[0] + sign * 0.01, _pf[0];
	Time << 0.05, 0.25, 0.5, 0.75, 0.95;
	computeSwingTrajectoryPolynomial(phase, swingTime, Pos, Time, pva);
	_p[0] = pva(0);
	_v[0] = pva(1);
	_a[0] = pva(2);
}

/*!
 * @brief foot trajectory plan, polynomial algorithm
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryPolynomial(T phase, T swingTime, Eigen::Matrix<T, 5, 1>& Pos, Eigen::Matrix<T, 5, 1>& Time, Vec3<T>& pva)
{
	Eigen::Matrix<T, 3, 3> coefA;
	Eigen::Matrix<T, 3, 1> coefC;

	T T0 = Time(1) - Time(0), T1 = Time(2) - Time(1), T2 = Time(3) - Time(2), T3 = Time(4) - Time(3), 
    q0 = Pos(0), q1 = Pos(1), q2 = Pos(2), q3 = Pos(3), q4 = Pos(4);
	coefA << 2 * (T0 + T1), T0, 0,
		T2, 2 * (T1 + T2), T1,
		0, T3, 2 * (T2 + T3);
	T v0 = 0;
	T v4 = 0;
	coefC(0) = 3 / (T0*T1)*(pow(T0, 2)*(q2 - q1) + pow(T1, 2)*(q1 - q0)) - T1 * v0;
	coefC(1) = 3 / (T1*T2)*(pow(T1, 2)*(q3 - q2) + pow(T2, 2)*(q2 - q1));
	coefC(2) = 3 / (T2*T3)*(pow(T2, 2)*(q4 - q3) + pow(T3, 2)*(q3 - q2)) - T2 * v4;
	Eigen::Matrix<T, 3, 1> vel123;
	vel123 = coefA.inverse()*coefC;

	Eigen::Matrix<T, 5, 1> vel;
	vel << 0, vel123(0), vel123(1), vel123(2), 0;

	T x = phase;
	T p, v, a;
	p = Pos(0); v = 0; a = 0;
	if (x >= Time(4))
	{
		p = Pos(4);
	}

	for (int i = 0; i < 4; i++)
	{
		if (x < Time(i + 1) && x >= Time(i))
		{
			T ti = Time(i + 1) - Time(i);
			T a0 = Pos(i);
			T a1 = vel(i);
			T a2 = 1 / ti * (3 * (Pos(i + 1) - Pos(i)) / ti - 2 * vel(i) - vel(i + 1));
			T a3 = 1 / pow(ti, 2)*(2 * (Pos(i) - Pos(i + 1)) / ti + vel(i) + vel(i + 1));
			p = a0 + a1 * (x - Time(i)) + a2 * pow((x - Time(i)), 2) + a3 * pow(x - Time(i), 3);
			v = a1 + 2 * a2*(x - Time(i)) + 3 * a3*pow(x - Time(i), 2);
			a = 2 * a2 + 6 * a3*(x - Time(i));
		}
	}
	pva(0) = p;
	pva(1) = v / ((Time(4) - Time(0))*swingTime);
	pva(2) = a / pow((Time(4) - Time(0))*swingTime, 2);
}
/// Add End

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;