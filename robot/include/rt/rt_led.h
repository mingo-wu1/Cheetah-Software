/*!
 * @file rt_led.h
 * @brief led drivers
 * @author hanyuanqiang
 */

#ifndef _rt_led
#define _rt_led

#include <string>

enum RtLedColour {
    COLOUR_RED = 0,
    COLOUR_ORANGE = 1,
    COLOUR_YELLOW = 2,
    COLOUR_GREEN = 3,
    COLOUR_CYAN = 4,
    COLOUR_BLUE = 5,
    COLOUR_PURPLE = 6,
    COLOUR_WHITE = 7,
    COLOUR_BLACK = 8,
};

class RtLed
{
public:
    RtLed();
    ~RtLed();
    std::string RtLedDisplay(RtLedColour ledColour);
    std::string RtLedDisplay(int ledColour);
private:
    std::string GetUartName();
    void RtLedInit(std::string uartName);
    int fd;
};

#endif
