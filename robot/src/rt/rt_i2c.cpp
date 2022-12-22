/*
 * @Author: wuchunming
 * @Date: 2021-12-09 21:20:32
 * @LastEditTime: 2021-12-24 15:33:46
 * @LastEditors: Please set LastEditors
 * @Description: i2c communication for robot
 * @FilePath: /Cheetah/robot/src/rt/rt_i2c.cpp
 */

#include "rt/rt_i2c.h"

RtI2C::RtI2C() :
  _bIsOk(false),
  _logger("RtI2C")
{

}

RtI2C::~RtI2C() 
{
  close(this->fd); 
}

/*
 * @description: init i2c
 * @param {int, int} i2c_bus_num, i2c_addr
 * @return: -1: write failed
 *           0: write success
 */
int RtI2C::Init(int bus, int addr) {
  // open i2c device
  std::string i2c_dev = "/dev/i2c-";
  i2c_dev += std::to_string(bus);
  int result = -1;
  this->fd = open(i2c_dev.c_str(), O_RDWR);
  if (this->fd < 0) {
    QUADRUPED_INFO(_logger, "open i2c failed\n");
    return result;
  }

  // set i2c device
  if (ioctl(this->fd, I2C_SLAVE, addr) < 0) {
    QUADRUPED_INFO(_logger, "set i2c slave addr failed\n");
    return result;
  }

  unsigned char step1[3] = {0b00000100, 0b00000000, 0b00001111}; // addr msb pin
  result = Write(step1, 3);
  if (result < 0) {
    QUADRUPED_INFO(_logger, "write step1 failed\n");
    return result;
  }

  unsigned char step2[3] = {0b00000010, 0b00000010, 0b00001111}; // addr msb pin
  result = Write(step2, 3);
  if (result < 0) {
    QUADRUPED_INFO(_logger, "write step2 failed\n");
    return result;
  }

  unsigned char step3[1] = {0b01000000};
  result = Write(step3, 1);
  if (result < 0) {
    QUADRUPED_INFO(_logger, "write step3 failed\n");
    return result;
  }

  _bIsOk = true;
  return result;
}

bool RtI2C::IsOk(){
  return _bIsOk;
}
/*
 * @description: write data to i2c device
 * @param: data: data to write
 * @param: len: length of data
 * @return: -1: write failed
 *          0: write success
 */
int RtI2C::Write(const void *buf, int len) {
  int ret = -1;
  if(this->fd >= 0){
    ret = write(this->fd, buf, len);
    if (ret < 0) {
      QUADRUPED_INFO(_logger, "write i2c failed\n");
    }
  }
  return ret;
}

/*
 * @description: read data from i2c device
 * @param: data: data to read
 * @param: len: length of data
 * @return: -1: read failed
 *          0: read success
 */
int RtI2C::Read(void *buf, int len) {
  int ret = -1;
  if(this->fd >= 0){
    ret = read(this->fd, buf, len);
    if (ret < 0) {
      QUADRUPED_INFO(_logger, "read i2c failed\n");
    }
  }
  return ret;
}

void RtI2C::ConvertAd5593R(unsigned char *src, int16_t *des, int des_len) {
  if(this->fd >= 0){
    for (int i = 0; i < des_len; ++i) {
      des[i] = (src[2 * i] << 8) + src[2 * i + 1];
    }
  }
  // printf("p: %d %d %d %d\n", des[0], des[1], des[2], des[3]);
}
