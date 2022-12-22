//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// explicitly request the C++17 version of the interface
#define OPENZEN_CXX17
#include "OpenZen.h"

#include <iostream>

using namespace zen;

/**
 * This example demonstrates the C++17 interface of the OpenZen library.
 */
int main(int argc, char* argv[])
{
    // enable resonable log output for OpenZen
    ZenSetLogLevel(ZenLogLevel_Info);

    // create OpenZen Clien
    auto [clientError, client] = make_client();

    if (clientError) {
        std::cout << "Cannot create OpenZen client" << std::endl;
        return clientError;
    }

    // connect to sensor on IO System by the sensor name
    auto [obtainError, sensor] = client.obtainSensorByName("SiUsb", "lpmscu2000573");
    if (obtainError)
    {
        std::cout << "Cannot connect to sensor" << std::endl;
        client.close();
        return obtainError;
    }

    // check that the sensor has an IMU component
    auto pImu = sensor.getAnyComponentOfType(g_zenSensorType_Imu);

    if (!pImu)
    {
        std::cout << "Connected sensor has no IMU" << std::endl;
        client.close();
        return ZenError_WrongSensorType;
    }

    // readout up to 200 samples from the IMU
    for (int i = 0; i < 200; i++) {
        auto event = client.waitForNextEvent();
        if (event->component.handle == pImu->component().handle) {
            std::cout << "> Acceleration: \t x = " << event->data.imuData.a[0]
                << "\t y = " << event->data.imuData.a[1]
                << "\t z = " << event->data.imuData.a[2] << std::endl;
            std::cout << "> Gyro: \t\t x = " << event->data.imuData.g[0]
                << "\t y = " << event->data.imuData.g[1]
                << "\t z = " << event->data.imuData.g[2] << std::endl;
        }
    }

    client.close();
    std::cout << "Sensor connection closed" << std::endl;
    return 0;
}