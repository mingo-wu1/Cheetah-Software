/*
 * @Author: your name
 * @Date: 2021-12-10 15:37:46
 * @LastEditTime: 2021-12-10 15:37:47
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /t/Cheetah/third-party/i2c/example/main_ad5593r.c
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long int uint64_t;

static volatile int keepRunning = 1;
void sig_handler( int sig )
{
    if ( sig == SIGINT)
    {
        keepRunning = 0;
    }
}

int main(int argc, char** argv)
{
    int i2cFD;
    if((i2cFD = open("/dev/i2c-5", O_RDWR)) < 0) {
        printf("Can't open i2c channel\n");
        return -1;
    }
 
    if(ioctl(i2cFD, 0x0703, 0x11) < 0) { // i2c_slave=17950x0703, i2c_bus_addr=0x11
        printf("IO_CTRL failed on the i2c\n");
        return -1;
    }

    uint8_t buff[3];
    buff[0] = 0b00000100; // addr
    buff[1] = 0b00000000; // msb
    buff[2] = 0b00001111; // pin
    int result = write(i2cFD, buff, 3);
    if(result < 0){
        printf("write fail.\n");
        return result;
    }

    buff[0] = 0b00000010; // addr
    buff[1] = 0b00000010; // msb
    buff[2] = 0b00001111; // pin
    result = write(i2cFD, buff, 3);
        if(result < 0){
        printf("write fail.\n");
        return result;
    }

    uint8_t buff2[1] = {0b01000000};
    result = write(i2cFD, buff2, 1);
    if(result < 0){
        printf("write fail.\n");
        return result;
    }
    
    uint8_t data[8] = {0};
    uint32_t regVal[4] = {0};
    while(keepRunning){
        int result = read(i2cFD, &data, 8);
        if(result < 0){
            printf("read fail.\n");
            return result;
        }
        
        for(size_t i=0; i<4; ++i){
            regVal[i] = (data[2*i] << 8) + data[2*i+1];
        }
        printf("p: %d %d %d %d\n", regVal[0], regVal[1], regVal[2], regVal[3]);

        usleep(100000);
    }
    close(i2cFD);
}