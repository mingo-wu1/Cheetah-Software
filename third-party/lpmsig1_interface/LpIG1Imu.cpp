#include "LpIG1Imu.h"
#include <unistd.h>
#include <time.h>
#define PORT_PREFIX "/dev/ttyUSB"

LpIG1Imu::LpIG1Imu()
{
    // Create LpmsIG1 object
    sensor1 = IG1Factory();
    sensor1->setVerbose(VERBOSE_INFO);
    sensor1->setAutoReconnectStatus(true);    
}

LpIG1Imu::~LpIG1Imu()
{
    sensor1->release();
}

void LpIG1Imu::init(u32 port, u32 baud_rate)
{
    printf("[LPIG1 IMU] Open port %d, baud rate %d\n", port, baud_rate);
    // Connects to sensor
    struct timespec time1, time2;// time init
    clock_gettime(CLOCK_MONOTONIC,&time1);//start time
    while(!(sensor1->connect(std::string(PORT_PREFIX)+std::to_string(port), static_cast<int>(baud_rate)))){
        clock_gettime(CLOCK_MONOTONIC,&time2);//end time
        if(((time2.tv_sec*1000.0+time2.tv_nsec/1000000.0) - (time1.tv_sec*1000.0+time1.tv_nsec/1000000.0))>=3000){//3000 ms
	    this->sensor1->release();
            throw std::runtime_error("Error connecting to sensor\n");
        }
    }

    printf("[LPIG1 IMU] Port open. Mode setup...\n");
    mode_setup();
    printf("[LPIG1 IMU] Get info...\n");
    get_device_info();
    // printf("[LPIG1 IMU] Self test...\n");
    // self_test();
    // printf("[LPIG1 IMU] Basic report...\n");
    // basic_report();
    // printf("[LPIG1 IMU] Zero Gyro...\n");
    // zero_gyro();
    printf("[LPIG1 IMU] Setup IMU...\n");
    setup_streaming();
    // printf("[LPIG1 IMU] Enable Data...\n");
    // enable();
}

void LpIG1Imu::setup_streaming(){
    setStreamingMode();
}

void LpIG1Imu::enable(){
}

bool LpIG1Imu::getImuData(){
    cmdGotoCommandMode();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    cmdGetImuData();
    return true;
}

void LpIG1Imu::cmdGetImuData()
{
    IG1Command cmd;
    cmd.command = GET_IMU_DATA;
    cmd.dataLength = 0;
    sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
}

void LpIG1Imu::get_device_info(){
    IG1SettingsI settings;
    sensor1->getSettings(settings);
    std::cout << settings.toString() << std::endl;
}

bool LpIG1Imu::tryInit(u32 port, u32 baud_rate)
{
    try
    {
        init(port, baud_rate);
    }
    catch (std::exception &e)
    {
        printf("[LPIG1 IMU] failed to initialize: %s\n", e.what());
        return false;
    }
    return true;
}

bool LpIG1Imu::setCommandMode()
{
    cmdGotoCommandMode();
    return true;
}

void LpIG1Imu::cmdGotoCommandMode()
{
    IG1Command cmd;
    cmd.command = GOTO_COMMAND_MODE;
    cmd.dataLength = 0;
    sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
}

bool LpIG1Imu::setStreamingMode()
{
    cmdGotoStreamingMode();
    return true;
}

void LpIG1Imu::cmdGotoStreamingMode()
{
    IG1Command cmd;
    cmd.command = GOTO_STREAM_MODE;
    cmd.dataLength = 0;
    sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
}

void LpIG1Imu::mode_setup()
{
    while (setCommandMode()==false)
        printf("failed to set command mode\n");
}

void LpIG1Imu::run()
{
    // for (u32 i = 0; i < 32; i++)
    update();
    // usleep(100);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
}

void LpIG1Imu::update()
{
    static bool runOnce = false;
    if (sensor1->getStatus() == STATUS_CONNECTED && sensor1->hasImuData())
    {
        if (!runOnce)
        {
            // publishIsAutocalibrationActive();
            runOnce = true;
        }
        IG1ImuDataI sd;
        sensor1->getImuData(sd);

        /* Fill the IMU message */

        // Fill angular velocity data
        // - scale from deg/s to rad/s
        gyro[0] = sd.gyroIAlignmentCalibrated.data[0] * 3.1415926 / 180;
        gyro[1] = sd.gyroIAlignmentCalibrated.data[1] * 3.1415926 / 180;
        gyro[2] = sd.gyroIAlignmentCalibrated.data[2] * 3.1415926 / 180;

        // Fill linear acceleration data
        acc[0] = -sd.accCalibrated.data[0] * 9.81;
        acc[1] = -sd.accCalibrated.data[1] * 9.81;
        acc[2] = -sd.accCalibrated.data[2] * 9.81;

        // Fill orientation quaternion
        quat[0] = sd.quaternion.data[0];
        quat[1] = -sd.quaternion.data[1];
        quat[2] = -sd.quaternion.data[2];
        quat[3] = -sd.quaternion.data[3];

	//std::cout<<"[IG1 TEST INFO] quat[0]:"<<quat[0]<<std::endl;
    }
}
