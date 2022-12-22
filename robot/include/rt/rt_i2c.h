/*
 * @Author: wuchunming
 * @Date: 2021-12-09 21:20:44
 * @LastEditTime: 2021-12-24 15:02:08
 * @LastEditors: Please set LastEditors
 * @Description: i2c communication for robot
 * @FilePath: /Cheetah/robot/include/rt/rt_i2c.h
 */
#ifndef RT_I2C_H
#define RT_I2C_H

#include <unistd.h>
#include "sys/ioctl.h"
#include "stdio.h"
#include "fcntl.h"
#include "linux/i2c-dev.h"
#include "Logger/Logger.h"
#include <string.h>

class RtI2C
{
    public:
        RtI2C();

        ~RtI2C();

        /*
            * @description: init i2c
            * @param {int, int} i2c_bus_num, i2c_addr
            * @return: -1: write failed
            *           0: write success
            */
        int Init(int bus, int addr);

        bool IsOk();
        /*
            * @description: write data to i2c device
            * @param: data: data to write
            * @param: len: length of data
            * @return: -1: write failed
            *          0: write success
            */   
        int Write(const void *buf, int len);

        /*
            * @description: read data from i2c device
            * @param: data: data to read
            * @param: len: length of data
            * @return: -1: read failed
            *          0: read success
            */
        int Read(void *buf, int len);
        
        void ConvertAd5593R(unsigned char *src, int16_t *des, int des_len);

    private:
        int fd;
        bool _bIsOk = false;
        BZL_QUADRUPED::Logger _logger;        
};

#endif
