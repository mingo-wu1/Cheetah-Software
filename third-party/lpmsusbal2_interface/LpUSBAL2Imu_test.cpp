//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "LpUSBAL2Imu.h"
#include <unistd.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    LpUSBAL2Imu imu;
    if(imu.initImu())
    {
        std::cout << "IMU init successed!" <<std::endl;
    }
    else
    {
        std::cout << "IMU init failed!!" <<std::endl;
        return 1;
    }
    bool whileFlag = true;
    std::string  line;
    while (whileFlag)
    {
        //imu.update();
        std::cout << " - 'q' to quit;" << std::endl;
        if (std::getline(std::cin, line))
        {
            if (line == "q")
                whileFlag = false;
        }
    }
    return 0;
}
