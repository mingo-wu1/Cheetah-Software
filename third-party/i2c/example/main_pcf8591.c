#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "i2c.h"

static volatile int keepRunning = 1;
void sig_handler( int sig )
{
    if ( sig == SIGINT)
    {
        keepRunning = 0;
    }
}

int main(int argc, char **argv){

    signal( SIGINT, sig_handler );

    unsigned int bus_num = 5;
    /* Open i2c bus */
    int bus;
    char bus_name[32];
    memset(bus_name, 0, sizeof(bus_name));

    if (snprintf(bus_name, sizeof(bus_name), "/dev/i2c-%u", bus_num) < 0) {
        fprintf(stderr, "Format i2c bus name error!\n");
        exit(-3);
    }

    if ((bus = i2c_open(bus_name)) == -1) {
        fprintf(stderr, "Open i2c bus:%s error!\n", bus_name);
        exit(-3);
    }

    unsigned short addr = 0x48;
    unsigned int page_bytes = 8;
    unsigned int iaddr_bytes = 1;

    /* Init i2c device */
    I2CDevice device;
    memset(&device, 0, sizeof(device));
    i2c_init_device(&device);

    device.bus = bus;
    device.addr = addr;
    device.page_bytes = page_bytes;
    device.iaddr_bytes = iaddr_bytes;

    size_t i = 0;
    ssize_t ret = 0;
    unsigned char buf[1];
    size_t buf_size = sizeof(buf);
    memset(buf, 0, buf_size);

    unsigned int AIN0 = 0x40;
    while(keepRunning){
        ret = i2c_ioctl_read(&device, AIN0, buf, 1);
        if (ret == -1 || (size_t)ret != buf_size)
        {

            fprintf(stderr, "Read i2c error!\n");
            exit(-5);
        }
	/* Print read result */
        fprintf(stdout, "Read data:\n");
        printf(" %d\n",(int)buf[0]);
	usleep(100000);
    }

    i2c_close(bus);
    return 0;
}
