/*! @file LeastSquare.h
 *  @brief least square
 *
 *
 */

#ifndef PROJECT_LEASTSQUARE_H
#define PROJECT_LEASTSQUARE_H

#include <cmath>

template <typename T>
class LeastSquare {
 public:
    //LeastSquare();
    T getY(T x){return a * x + b;};
    T computeY(T*, T*, int, T);
    T getA(){return a;};
    T getB(){return b;};
    void computeLeastSquare(T*, T*, int);
 private:
    T a = 0, b = 0;
};

template <typename T>
void LeastSquare<T>::computeLeastSquare(T* xList, T* yList, int length){
    T t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    for (int i = 0; i < length; i++){
        t1 = t1 + xList[i] * xList[i];
        t2 = t2 + xList[i];
        t3 = t3 + xList[i] * yList[i];
        t4 = t4 + yList[i];    
    }
    a = (t3 * length - t2 * t4) / (t1 * length - t2 * t2);
    b = (t1 * t4 - t2 * t3) / (t1 * length - t2 * t2);
}

template <typename T>
T LeastSquare<T>::computeY(T* xList, T* yList, int length, T x){
    computeLeastSquare(xList, yList, length);
    return a * x + b;
}
#endif