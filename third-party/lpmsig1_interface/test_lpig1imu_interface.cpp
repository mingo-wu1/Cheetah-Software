#include "LpIG1Imu.h"
#include <iostream>
#include <unistd.h>

int main()
{
    LpIG1Imu lpIG1;
	lpIG1.tryInit(1,115200);
	while(1)
	{
    	lpIG1.run();
	//	usleep(3000000);
	}
    return 0;
}
