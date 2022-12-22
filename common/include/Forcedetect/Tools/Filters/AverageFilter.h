/*! @file AverageFilter.h
 *  @brief A simple average filter
 *
 *
 */

#ifndef PROJECT_AVERAGEFILTER_H
#define PROJECT_AVERAGEFILTER_H

#include <cmath>
#include "../Fitting/LeastSquare.h"
#include "BaseFilter.h"

#define BF BaseFilter<T>

template <typename T>
class AverageFilter :public BaseFilter<T>{
 public:
    AverageFilter();
    AverageFilter(int);
    void update(T);
    void updateData(T);
    void updateDataDeviation(T);
    T getFilteredData(){return BF::_filtered_data[BF::_windows - 1];}
    T getFilteredDataDeriviation(){return BF::_filtered_deviation_data;}
    
 private:
    LeastSquare<T> _least_square;
};

template <typename T>
AverageFilter<T>::AverageFilter(){
     BF::_windows = 4;
}

template <typename T>
AverageFilter<T>::AverageFilter(int filtered_num){
    BF::_windows = filtered_num;
}

template <typename T>
void AverageFilter<T>::updateData(T new_raw_date){
    for (int i = 1; i < BF::_windows; i ++)
    {
        BF::_raw_data[i-1] = BF::_raw_data[i];
    }
    BF::_raw_data[BF::_windows - 1] = new_raw_date;

    for (int i = 1; i < BF::_windows; i++){
        BF::_filtered_data[i-1] = BF::_filtered_data[i];
    }

    BF::_filtered_data[BF::_windows - 1] = 0;
    for (int i = 0; i < BF::_windows; i++)
    {
        BF::_filtered_data[BF::_windows - 1] = BF::_filtered_data[BF::_windows - 1] + BF::_raw_data[i];
    }
    BF::_filtered_data[BF::_windows - 1] = BF::_filtered_data[BF::_windows - 1] / BF::_windows;

    _least_square.computeLeastSquare(BF::_xList, BF::_filtered_data, BF::_windows);
    BF::_filtered_deviation_data = _least_square.getA();
}

template <typename T>
void AverageFilter<T>::updateDataDeviation(T new_raw_date){
    updateData(new_raw_date);
}

template <typename T>
void AverageFilter<T>::update(T new_raw_date){
    updateData(new_raw_date);
}

#endif