/*! @file PolyfitDelay.h
 *  @fitting the curve without delay
 *
 */

#ifndef POLYFITDELAY_H
#define POLYFITDELAY_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include "BaseFilter.h"

#define BF BaseFilter<T>

template <typename T>
class PolyfitDelay: public BaseFilter<T>{
  public:
    Polyfit(){};
	Polyfit(int windows):BF(windows){};
	Polyfit(int windows, int order):BF(windows),_order(order){};
	void update(T);
    void updateData(T);
    void updateDataDeviation(T);
	T getFilteredData(){return BF::_filtered_data[BF::_windows - 1];}
    T getFilteredDataDeriviation(){return BF::_filtered_deviation_data;}
    double polyeval(Eigen::VectorXd coeffs, double x);
	Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

  private:
	int _order = 2;

};

template <typename T>
double Polyfit<T>::polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) { result += coeffs[i] * pow(x, i); }
	return result;
}

template <typename T>
Eigen::VectorXd Polyfit<T>::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);
	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}
	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}
	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

template <typename T>
void Polyfit<T>::updateData(T new_raw_date){
	for (int i = 1; i < BF::_windows; i ++)
    {
        BF::_raw_data[i-1] = BF::_raw_data[i];
    }
    BF::_raw_data[BF::_windows - 1] = new_raw_date;

    for (int i = 1; i < BF::_windows; i++){
        BF::_filtered_data[i-1] = BF::_filtered_data[i];
	}

	Eigen::VectorXd X(BF::_windows);
	Eigen::VectorXd Y(BF::_windows);
	//Eigen::VectorXd coeffs(BF::_windows);
    for (int i = 0; i < BF::_windows; i++){
		X[i] = BF::_xList[i];
		Y[i] = BF::_raw_data[i];
	}
	auto coeffs = polyfit(X, Y, _order);
	std::cout << "X " << X << std::endl;
	std::cout << "Y " << Y << std::endl;
	double out = polyeval(coeffs, X[BF::_windows - 1]);
	std::cout << "out " << out << std::endl;
	BF::_filtered_data[BF::_windows - 1] = (T)out;
	double Sum = 0;
	for (int i = 1; i < _order + 1; i++)
	{
		Sum = Sum + coeffs[i] * i * pow(X[BF::_windows - 1], i - 1);
	}
	BF::_filtered_deviation_data = (T)Sum;
}

template <typename T>
void Polyfit<T>::update(T new_raw_date){
	updateData(new_raw_date);
}

template <typename T>
void Polyfit<T>::updateDataDeviation(T new_raw_date){
	updateData(new_raw_date);
}

#endif