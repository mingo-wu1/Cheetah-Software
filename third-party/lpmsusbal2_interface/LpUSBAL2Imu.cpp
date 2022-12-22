#include "LpUSBAL2Imu.h"
#define SHOW_DATA
using namespace zen;
using namespace std;

void pollLoop(std::reference_wrapper<ZenClient> client);
void addDiscoveredSensor(const ZenEventData_SensorFound& desc);

std::vector<ZenSensorDesc> _discoveredSensors;
std::condition_variable _discoverCv;
std::mutex _discoverMutex;
std::atomic<uintptr_t> _imuHandle;
std::atomic<uintptr_t> _gnssHandle;
std::atomic_bool g_terminate(false);
Vec3<float> g_gyro;
Vec3<float> g_acc;
Vec3<float> g_rpy;
Vec4<float> g_quat;
std::mutex some_mutex;

LpUSBAL2Imu::LpUSBAL2Imu()
{
    ZenSetLogLevel(ZenLogLevel_Info);
    _imuHandle = 0;
    _gnssHandle = 0;
}

LpUSBAL2Imu::~LpUSBAL2Imu()
{
    g_terminate = true;
    if(_client != NULL)
    _client->close();
    _client = NULL;
    _pollingThread.join();
    // _clientPair = nullptr;
}


bool LpUSBAL2Imu::initImu()
{
    static auto clientPair = make_client();
    // _clientPair = &clientPair;
    static auto& clientError = clientPair.first;
    static auto& client = clientPair.second;
    _client = &client;
    if(clientError)
        return false;
    _pollingThread = std::thread(&pollLoop, std::ref(client));
    
    std::cout << "Listing sensors:" << std::endl;
    ZenError error = client.listSensorsAsync();
    if(error != ZenError_None)
    {
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }

    std::unique_lock<std::mutex> lock(_discoverMutex);
    _discoverCv.wait(lock);

    if (_discoveredSensors.empty())
    {
        std::cout << " -- no sensors found -- " << std::endl;
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }
    long int cur_idx = -1;
    for (unsigned idx = 0; idx < _discoveredSensors.size(); ++idx)
    {
        std::cout << idx << ": " << _discoveredSensors[idx].name << " ("  << _discoveredSensors[idx].ioType << ")" << std::endl;
        std::string cur_name = _discoveredSensors[idx].name;
        if(cur_name.substr(0,7) == "LPMSUA2")
        {
            cur_idx = idx;
        }
    }
        
    if(cur_idx < 0)
    {
        std::cout << " -- no sensors found -- " << std::endl;
        return false;
    }
    static auto sensorPair = client.obtainSensor(_discoveredSensors[cur_idx]);
    static auto& obtainError = sensorPair.first;
    static auto& sensor = sensorPair.second;
    if (obtainError)
    {
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }
    static auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    static auto& hasImu = imuPair.first;
    static auto imu = imuPair.second;

    if (!hasImu)
    {
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }
    // store the handle to the IMU to identify data coming from the imu
    // in our data processing thread
    _imuHandle = imu.component().handle;

    // Get a string property
    static auto sensorModelPair = sensor.getStringProperty(ZenSensorProperty_SensorModel);
    static auto & sensorModelError = sensorModelPair.first;
    static auto & sensorModelName = sensorModelPair.second;
    if (sensorModelError) {
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }
    std::cout << "Sensor Model: " << sensorModelName << std::endl;


    // check if a Gnss component is present on this sensor
    static auto gnssPair = sensor.getAnyComponentOfType(g_zenSensorType_Gnss);
    static auto& hasGnss = gnssPair.first;
    static auto gnss = gnssPair.second;

    if (hasGnss)
    {
        // store the handle to the Gnss to identify data coming from the Gnss
        // in our data processing thread
        _gnssHandle = gnss.component().handle;
        std::cout << "Gnss Component present on sensor" << std::endl;

        // enable this code for RTK forwarding from network
        /*
        if (gnss.forwardRtkCorrections("RTCM3Network", "192.168.1.117", 9000) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */

        // enable this code for RTK forwarding from serial port
        /*
        if (gnss.forwardRtkCorrections("RTCM3Serial", "COM11", 57600) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */
    }
    // Enable sensor streaming
    auto error1 = imu.setBoolProperty(ZenImuProperty_StreamData, true);
    if (error1 != 0)
    {
        g_terminate = true;
        _client->close();
        _pollingThread.join();
        return false;
    }
    return true;
}

bool LpUSBAL2Imu::update()
{
   some_mutex.lock();
   gyro = g_gyro *(3.1415926535 / 180.0);
   acc = -9.81f * g_acc;
   rpy = g_rpy * (3.1415926535 / 180.0);
   quat = g_quat;
   quat[1] = -quat[1];
   quat[2] = -quat[2];
   quat[3] = -quat[3];
   some_mutex.unlock();
   return true;
}

void addDiscoveredSensor(const ZenEventData_SensorFound& desc)
{
    std::lock_guard<std::mutex> lock(_discoverMutex);
    _discoveredSensors.push_back(desc);
}

void pollLoop(std::reference_wrapper<ZenClient> client)
{
    unsigned int i = 0;
    while (!g_terminate)
    {
        const auto pair = client.get().waitForNextEvent();
        const bool success = pair.first;
        auto& event = pair.second;
        if (!success)
            break;

        if (!event.component.handle)
        {
            if (event.eventType == ZenEventType_SensorFound)
            {
                addDiscoveredSensor(event.data.sensorFound);
            }
            else if (event.eventType == ZenEventType_SensorListingProgress)
            {
                if (event.data.sensorListingProgress.progress == 1.0f)
                    _discoverCv.notify_one();
            }
        }
        else if (( _imuHandle > 0) && (event.component.handle == _imuHandle))
        {
            if (event.eventType == ZenEventType_ImuData)
            {
                some_mutex.lock();
                //Acceleration
                g_acc[0] = event.data.imuData.a[0];
                g_acc[1] = event.data.imuData.a[1];
                g_acc[2] = event.data.imuData.a[2];
                //Gyro
                g_gyro[0] = event.data.imuData.g[0];
                g_gyro[1] = event.data.imuData.g[1];
                g_gyro[2] = event.data.imuData.g[2];
                //Quaternion orientation data  w, x, y, z
                g_quat[0] = event.data.imuData.q[0];
                g_quat[1] = event.data.imuData.q[1];
                g_quat[2] = event.data.imuData.q[2];
                g_quat[3] = event.data.imuData.q[3];
                //RPY
                g_rpy[0] = event.data.imuData.r[0];
                g_rpy[1] = event.data.imuData.r[1];
                g_rpy[2] = event.data.imuData.r[2];
                some_mutex.unlock();
                
                if (i++ % 5000 == 0) {
                    #ifdef SHOW_DATA
					std::cout << "LPMS-USBAL2  working." << std::endl;
                    //std::cout << "Event type: " << event.eventType << std::endl;
     //               std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
     //               std::cout << "> Acceleration: \t x = " << event.data.imuData.a[0]
     //                   << "\t y = " << event.data.imuData.a[1]
     //                   << "\t z = " << event.data.imuData.a[2] << std::endl;
     //               std::cout << "> Gyro: \t\t x = " << event.data.imuData.g[0]
     //                   << "\t y = " << event.data.imuData.g[1]
     //                   << "\t z = " << event.data.imuData.g[2] << std::endl;
					//std::cout << "> rpy: \t\t x = " << event.data.imuData.r[0]
					//    << "\t y = " << event.data.imuData.r[1]
					//    << "\t z = " << event.data.imuData.r[2] << std::endl;
     //               std::cout << "> Quaternion: \t\t w = " << event.data.imuData.q[0]
     //                   << "\t x = " << event.data.imuData.q[1]
     //                   << "\t y = " << event.data.imuData.q[2]
     //                   << "\t z = " << event.data.imuData.q[3] << std::endl;
                    #endif
                }
            }
        }
        else if (( _gnssHandle > 0) && (event.component.handle == _gnssHandle))
        {
            if (event.eventType == ZenEventType_GnssData)
            {
                // std::cout << "Event type: " << event.eventType << std::endl;
                // std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
                // std::cout << "> GPS Fix: \t = " << event.data.gnssData.fixType << std::endl;
                // std::cout << "> Longitude: \t = " << event.data.gnssData.longitude
                //     << "   Latitude: \t = " << event.data.gnssData.latitude << std::endl;
                // std::cout << " > GPS Time " << int(event.data.gnssData.year) << "/"
                //     << int(event.data.gnssData.month) << "/"
                //     << int(event.data.gnssData.day) << " "
                //     << int(event.data.gnssData.hour) << ":"
                //     << int(event.data.gnssData.minute) << ":"
                //     << int(event.data.gnssData.second) << " UTC" << std::endl;

                // if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_None) {
                //     std::cout << " > RTK not used" << std::endl;
                // }
                // else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FloatAmbiguities) {
                //     std::cout << " > RTK used with float ambiguities" << std::endl;
                // }
                // else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FixedAmbiguities) {
                //     std::cout << " > RTK used with fixed ambiguities" << std::endl;
                // }
            }
        }
    }

    std::cout << "--- Exit polling thread ---" << std::endl;
}
