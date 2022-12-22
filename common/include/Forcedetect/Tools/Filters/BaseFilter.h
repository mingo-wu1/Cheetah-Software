/*! @file BaseFilter.h
 *  @base class
 *
 */

#ifndef BASEFILTER_H
#define BASEFILTER_H

template <typename T>
class BaseFilter{
  public:
    BaseFilter(){};
    BaseFilter(int windows){_windows = windows;};
    BaseFilter(int, int){};
    virtual void update(T) = 0;
    virtual void updateData(T) = 0;
    virtual void updateDataDeviation(T) = 0;
    virtual T getFilteredData() = 0;
    virtual T getFilteredDataDeriviation() = 0;
  protected:
    static const int num = 20;
    int _windows = 0;
    T _raw_data[num] = {0};
    T _filtered_data[num] = {0};
    T _xList[num] = {1,2,3,4,5,6,7,8,9,10};
    T _filtered_deviation_data = 0;
};

#endif