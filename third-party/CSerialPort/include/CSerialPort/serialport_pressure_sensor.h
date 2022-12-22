#ifndef SERIALPORT_PRESSURE_SENSOR_H
#define SERIALPORT_PRESSURE_SENSOR_H
#include <iostream>
#include <iomanip>
#include <sstream>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "CSerialPort/SerialPort_global.h"

namespace itas109
{
typedef union PressureDataFrame
{
    typedef char byte;
    typedef struct
    {
        byte head;
        byte cmd1;
        byte cmd2;
        byte sensor0[2];
        byte sensor1[2];
        byte sensor2[2];
        byte sensor3[2];
        byte check;
    } PDFrame;

    byte data[12];
    PDFrame frame;

} PressureDFrame;

struct SensorData
{
    virtual void SetData(char* bufData, int len) = 0;
    virtual const int* GetData() const = 0;
};

struct PressureData : public SensorData
{
#define COUNT 4
    typedef char byte;

private:
    int sensor[COUNT];
    PressureDFrame pdframe_;
    std::stringstream ss;

public:
    void SetData(char* bufData, int len)
    {
        //std::cout << "read data:";
        for (int i = 0; i < len; ++i)
        {
            //std::cout << std::hex << (int)(unsigned byte)bufData[i] << " ";
            pdframe_.data[i] = bufData[i];
        }
        //std::cout << std::dec << ", read size:" << len << std::endl;

        byte *s[COUNT]{pdframe_.frame.sensor0, 
                       pdframe_.frame.sensor1, 
                       pdframe_.frame.sensor2,
                       pdframe_.frame.sensor3};

        for (size_t i = 0; i < COUNT; ++i){
		ss << std::hex << (int)(unsigned byte)s[i][0] << (int)(unsigned byte)s[i][1]<<std::endl;
		sensor[i] = stoi(ss.str());
		ss.str("");
		//sensor[i] = ((sensor[i] | s[i][0]) << 8) | s[i][1];
	}
    }

    const int* GetData() const
    {
        return sensor;
    }
};

class IOSlot : public has_slots<>
{
public:
    IOSlot(itas109::CSerialPort &sp, itas109::SensorData &data) : readLen_(-1), serialPort_(sp), sensorData_(&data) {}
    IOSlot(itas109::CSerialPort &sp) : readLen_(-1), serialPort_(sp){}

    void OnWriteMsg()
    {
        // read data        
        readLen_ = serialPort_.readAllData(byteBufData_);
        if (readLen_ > 0)
        {
            static int readCounts = 0;
            ++readCounts;
            byteBufData_[readLen_] = '\0';

            // set data frame
            if(sensorData_)
                sensorData_->SetData(byteBufData_, readLen_);

            // frame test
            // test();

            // close port
            if (readCounts > 700)
            {
                std::cout << "close serial port when receive count > 700" << std::endl;
                serialPort_.close();
            }
            else
            {
                // serialPort_.writeData(byteBufData_, readLen_);
            }
        }
        else
        {
        }
    }

    void SetSensorData(itas109::SensorData &data){
        sensorData_ = &data;
    }

    const itas109::SensorData* GetSensorData() const{
        return sensorData_ ? sensorData_ : nullptr;
    }

private:
    void test()
    {        
        // tempSensors.SetData(pdframe_.frame.sensor0, pdframe_.frame.sensor1, pdframe_.frame.sensor2,
        //                     pdframe_.frame.sensor3);

        // std::cout << std::hex << (int)(unsigned byte)pdframe_.frame.head << " "
        //           << (int)(unsigned byte)pdframe_.frame.cmd1 << " " << (int)(unsigned byte)pdframe_.frame.cmd2 << " "
        //           << std::dec << (int)(unsigned short)tempSensors.GetData()[0] << " "
        //           << (int)(unsigned short)tempSensors.GetData()[1] << " "
        //           << (int)(unsigned short)tempSensors.GetData()[2] << " "
        //           << (int)(unsigned short)tempSensors.GetData()[3] << " " << std::hex
        //           << (int)(unsigned byte)pdframe_.frame.check << " ";
        // std::cout << std::endl;
    }

private:
    typedef char byte;
    byte byteBufData_[512];
    PressureDFrame pdframe_;
    int readLen_;
    itas109::CSerialPort serialPort_;
    SensorData *sensorData_;
};
} // namespace itas109

#endif // SERIALPORT_PRESSURE_SENSOR_H