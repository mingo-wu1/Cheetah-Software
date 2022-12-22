#include <iostream>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "CSerialPort/SerialPort_global.h"

typedef union PressureDataFrame{
    typedef char byte;
    typedef struct{
        byte head;
        byte cmd1;
        byte cmd2;
        byte sensor0[2];
        byte sensor1[2];
        byte sensor2[2];
        byte sensor3[2];
        byte check;
    }PDFrame;

    byte data[12];
    PDFrame frame;

}PressureDFrame;

struct PressureData{
#define COUNT 4
typedef char byte;
private:
    short sensor[COUNT];

public:
    void SetData(byte* s0, byte* s1, byte* s2, byte* s3)
    {
        byte* s[COUNT]{s0, s1, s2, s3};
        for(size_t i = 0; i < COUNT; ++i)
            sensor[i] = ((sensor[i] | s[i][0]) << 8 )| s[i][1];
    }

    const short* GetData() const
    {
        return sensor;
    }
};

class IOSlot : public has_slots<>
{
public:
    IOSlot(itas109::CSerialPort &sp):serialPort_(sp),readLen_(-1){}

    void OnWriteMsg()
    {
        // read data
        readLen_ = serialPort_.readAllData(byteBufData_);
        if(readLen_ > 0)
        {
            static int readCounts = 0;
            ++readCounts;
            byteBufData_[readLen_] = '\0';

            // set data frame
            std::cout<<"read data:";            
            for(int i = 0; i < readLen_; ++i){
                std::cout<<std::hex<<(int)(unsigned byte)byteBufData_[i]<<" ";
                pdframe_.data[i] = byteBufData_[i];           
            }
            std::cout<<std::dec<<", read size:"<<readLen_<<
            ", read counts:"<<readCounts<<std::endl;
            
            // frame test
            test();

            // close port          
            if(readCounts > 7)
            {
                std::cout << "close serial port when receive count > 7" << std::endl;
				serialPort_.close();
            }
            else
            {
                // serialPort_.writeData(byteBufData_, readLen_);
            }    
        }
        else
        {}
    }

    void test()
    {
        PressureData tempSensors;
        tempSensors.SetData(pdframe_.frame.sensor0, pdframe_.frame.sensor1, pdframe_.frame.sensor2, pdframe_.frame.sensor3);

        std::cout<<std::hex
        <<(int)(unsigned byte)pdframe_.frame.head<<" "
        <<(int)(unsigned byte)pdframe_.frame.cmd1<<" "
        <<(int)(unsigned byte)pdframe_.frame.cmd2<<" "
        <<std::dec
        <<(int)(unsigned short)tempSensors.GetData()[0]<<" "
        <<(int)(unsigned short)tempSensors.GetData()[1]<<" "
        <<(int)(unsigned short)tempSensors.GetData()[2]<<" "
        <<(int)(unsigned short)tempSensors.GetData()[3]<<" "
        <<std::hex
        <<(int)(unsigned byte)pdframe_.frame.check<<" ";
        std::cout<<std::endl;
    }

private:
    itas109::CSerialPort serialPort_;
    int readLen_{-1};
    typedef char byte;
    byte byteBufData_[512]{{0}};
    PressureDFrame pdframe_{{0}};
};

int main()
{
    // construct serialport and serialportinfo which for availableport
    int index = -1;
    std::string portName;    
    itas109::CSerialPort serialPort;
    std::vector<SerialPortInfo> availablePortsList;
    availablePortsList = itas109::CSerialPortInfo::availablePortInfos();
    IOSlot readSlot(serialPort);    

    std::cout << "Version : " << serialPort.getVersion() << std::endl << std::endl;
    std::cout << "availableFriendlyPorts : " << std::endl;
    for(size_t i = 0; i < availablePortsList.size(); ++i){
        std::cout<<i<<"-"<<availablePortsList[i].portName
        <<" "<<availablePortsList[i].description<<std::endl;
    }

    if(0 == availablePortsList.size())
    {
        std::cout<<"No valid port"<<std::endl;
    }
    else
    {   
        //serialport name selected by cin
        std::cout << std::endl;
        do
        {
            std::cout<<"Please input index of the port(0 - "
            <<availablePortsList.size() - 1<< " ) : " << std::endl;
            std::cin>>index;
            if(index >= 0 && index < static_cast<int>(availablePortsList.size()))
            {
                break;
            }
        }while(true);

        portName = availablePortsList[index].portName;
        std::cout<<"select port name:"<<portName<<std::endl;

        // serialport init and open
        serialPort.init(portName,
                        115200, 
                        itas109::Parity::ParityNone,
                        itas109::DataBits::DataBits8,
                        itas109::StopBits::StopOne,
                        itas109::FlowControl::FlowNone, 512);
        serialPort.open();
        if(serialPort.isOpened())
		{
			std::cout << "open " << portName << " success" << std::endl;	
		}
		else
		{
			std::cout << "open " <<  portName << " failed" << std::endl;
        }

        // only connect when data come and read, then write msg immediately
        serialPort.readReady.connect(&readSlot, &IOSlot::OnWriteMsg);
        // serialPort.writeData("itas109", 7);

        while(true);
    }
    return 0;
}

