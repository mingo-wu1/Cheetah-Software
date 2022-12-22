#include <string>
#include <map>
#include <stdio.h>
#include "lcm/lcm-cpp.hpp"
#include "gamepad_lcmt.hpp"
#include "gamecontroller.h"
#include "unistd.h"
#include <QtCore/QCoreApplication>
#include <thread>
#include <ctime>
#include <linux/input.h>
#include <linux/joystick.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/// Add Begin by wuchunming, 2021-03-22, add thread to cal time for gamepad offline
int xbox_open(const char *file_name);
int xbox_map_read(int xbox_fd);
void xbox_close(int xbox_fd);
struct timespec startTime, endTime;
#define OVERTIME 300.0
/// Add End

int main(int argc, char**argv)
{
    QCoreApplication application(argc, argv);
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;
    GameController gc{};
    
    std::thread mythread([&](){
        gamepad_lcmt g{};
        while (true)
        {
            usleep(100000);
            gc.updateGamepadCommand(g);
            lcm.publish("interface", &g);
        }
    });

    /// Add Begin by wuchunming, 2021-03-22, add thread to cal time for gamepad offline
    std::thread minitorThread([&](){
        int xbox_fd;
        int len;

        xbox_fd = xbox_open("/dev/input/js0");

        if(xbox_fd < 0)
            return -1;

        clock_gettime(CLOCK_MONOTONIC,&startTime);//the first start time 
        std::thread t([](){
            while(true){
                clock_gettime(CLOCK_MONOTONIC,&endTime);
                usleep(1*1000*1000);
                if(endTime.tv_sec-startTime.tv_sec>OVERTIME)
                    printf("--- 5min, gamepad offline ---\n");
            }
        });
        t.detach();

        while(true)
        {
            len = xbox_map_read(xbox_fd);
            clock_gettime(CLOCK_MONOTONIC,&startTime);//start time for thread loop
//            printf("--- gamepad online ---\n");

            if (len < 0)
            {
                usleep(10*1000);
                continue;
            }
        }

        xbox_close(xbox_fd);
        return 0;

    });
    minitorThread.detach();
    /// Add End

    return application.exec();
};

/// Add Begin by wuchunming, 2021-03-22, add thread to cal time for gamepad offline
int xbox_open(const char *file_name)
{
    int xbox_fd;

    xbox_fd = open(file_name, O_RDONLY);
    if (xbox_fd < 0)
    {
        perror("open");
        return -1;
    }

    return xbox_fd;
}

int xbox_map_read(int xbox_fd)
{
    int len;
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0)
    {
        perror("read");
        return -1;
    }

    return len;
}

void xbox_close(int xbox_fd)
{
    close(xbox_fd);
    return;
}
/// Add End
