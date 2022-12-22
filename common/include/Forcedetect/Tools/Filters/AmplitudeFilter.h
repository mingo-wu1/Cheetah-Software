/*! @file AmplitudeFilter.h
 *  @brief 0 or 1
 *
 *
 */

#ifndef PROJECT_AMPLITUDEFILTER_H
#define PROJECT_AMPLITUDEFILTER_H

#define BF BaseFilter<T>

template <typename T>
class AmplitudeFilter : public BaseFilter<T> {
 public:
   AmplitudeFilter(){};
   AmplitudeFilter(int windows):BF(windows){};
   AmplitudeFilter(int windows, T amplitude):BF(windows), _amplitude(amplitude){};
   void update(T);
   void updateData(T);
   void updateDataDeviation(T){};
   T getFilteredData(){return BF::_filtered_data[BF::_windows - 1];}
   T getFilteredDataDeriviation() {return 0;}
 private:
   T _amplitude = 35;

};

template <typename T>
void AmplitudeFilter<T>::updateData(T new_data){
    for (int i = 1; i < BF::_windows; i ++)
    {
        BF::_raw_data[i-1] = BF::_raw_data[i];
    }
    BF::_raw_data[BF::_windows - 1] = new_data;
    int k = 1;
    for (int i = 0; i < BF::_windows; i++){
        if (BF::_raw_data[i] < _amplitude){
            k = 0;
            break;
        }
    }
    if (k == 1){BF::_filtered_data[BF::_windows - 1] = 1;}
    else{BF::_filtered_data[BF::_windows - 1] = 0;}
}

template <typename T>
void AmplitudeFilter<T>::update(T new_data){
    updateData(new_data);
}

#endif