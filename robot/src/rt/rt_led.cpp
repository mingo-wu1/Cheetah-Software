/*!
 * @file rt_led.cpp
 * @brief led drivers
 * @author hanyuanqiang
 */

#include "rt/rt_led.h"

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>
#include <fstream>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

RtLed::RtLed()
    : fd(-1)
{
    std::string uartName;
    uartName = GetUartName();
    if (uartName != "")
    {
        RtLed::RtLedInit(uartName);
    }
}

RtLed::~RtLed()
{
    if (fd < 0)
    {
        return;
    }
    RtLedDisplay(COLOUR_BLACK);
    close(fd);
}

inline bool startsWith(std::string const &fullString, std::string const &start)
{
    if (fullString.length() >= start.length())
    {
        return (0 == fullString.compare(0, start.length(), start));
    }
    else
    {
        return false;
    }
}

inline bool endsWith(std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length())
    {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    }
    else
    {
        return false;
    }
}

std::string RtLed::GetUartName()
{
    std::ifstream propFileVendor;
    std::ifstream propFileProduct;
    std::string propLineVendor;
    std::string propLineProduct;

    std::ifstream propFile;
    std::string propLine;

    for (auto &usb_device : fs::directory_iterator("/sys/bus/usb/devices/"))
    {
        propFileVendor.open(usb_device.path() / "idVendor", std::ifstream::in);
        std::getline(propFileVendor, propLineVendor);
        propFileVendor.close();
        propFileProduct.open(usb_device.path() / "idProduct", std::ifstream::in);
        std::getline(propFileProduct, propLineProduct);
        propFileProduct.close();

        if ((propLineVendor != "10c4") || 
            ((propLineProduct != "ea60") &&(propLineProduct != "ea61")))
        {
            continue;
        }

        propFile.open(usb_device.path() / "serial", std::ifstream::in);
        std::getline(propFile, propLine);
        propFile.close();
        if (propLine != "0001")
        {
            continue;
        }

        for (auto &p : fs::directory_iterator(usb_device.path()))
        {
            // p is something like /sys/bus/usb/devices/1-6/1-6:1.0
            if (endsWith(p.path(), ":1.0"))
            {
                // found device folder, look for tty* Folder inside
                for (auto &deviceFolder : fs::directory_iterator(p.path()))
                {
                    auto lastFolder = deviceFolder.path().end();
                    if (deviceFolder.path().begin() != lastFolder)
                    {
                        lastFolder = --lastFolder;
                        if (startsWith(*lastFolder, "tty"))
                        {
                            // generate the file name on the system /dev folder
                            const auto deviceFileName = fs::path("/dev/") / std::string(*lastFolder);
                            return deviceFileName;
                        }
                    }
                }
            }
        }

    }

    return "";
}

void RtLed::RtLedInit(std::string uartName)
{
    struct termios newtio;

    fd = open((char *)(uartName.c_str()), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        return;
    }
    bzero( &newtio, sizeof( newtio ) );
    tcgetattr(fd, &newtio);
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW,&newtio);
}

std::string RtLed::RtLedDisplay(RtLedColour ledColour)
{
    uint8_t nLen = 0;
    uint8_t u8DisplayData[21] = {0xDD, 0x55, 0xEE, 0x00, 0x00, 0x00, 0x01, 0x00, 0x99, 
        0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0xAA, 0xBB};
    // uint8_t u8RecvData[255] = {0};

    if (fd < 0)
    {
        return "error";
    }

    switch (ledColour)
    {
        case COLOUR_RED:
            u8DisplayData[16] = 0xFF;
            u8DisplayData[17] = 0x00;
            u8DisplayData[18] = 0x00;
        break;
        case COLOUR_ORANGE:
            u8DisplayData[16] = 0xFF;
            u8DisplayData[17] = 0x7D;
            u8DisplayData[18] = 0x00;
        break;
        case COLOUR_YELLOW:
            u8DisplayData[16] = 0xFF;
            u8DisplayData[17] = 0xFF;
            u8DisplayData[18] = 0x00;
        break;
        case COLOUR_GREEN:
            u8DisplayData[16] = 0x00;
            u8DisplayData[17] = 0xFF;
            u8DisplayData[18] = 0x00;
        break;
        case COLOUR_CYAN:
            u8DisplayData[16] = 0x00;
            u8DisplayData[17] = 0xFF;
            u8DisplayData[18] = 0xFF;
        break;
        case COLOUR_BLUE:
            u8DisplayData[16] = 0x00;
            u8DisplayData[17] = 0x00;
            u8DisplayData[18] = 0xFF;
        break;
        case COLOUR_PURPLE:
            u8DisplayData[16] = 0xFF;
            u8DisplayData[17] = 0x00;
            u8DisplayData[18] = 0xFF;
        break;
        case COLOUR_WHITE:
            u8DisplayData[16] = 0xFF;
            u8DisplayData[17] = 0xFF;
            u8DisplayData[18] = 0xFF;
        break;
        case COLOUR_BLACK:
        break;
        default:
        break;
    }

    nLen = write(fd, (char *)u8DisplayData, sizeof(u8DisplayData));
    // nLen = read(fd, (char *)u8RecvData, sizeof(u8RecvData));
    (void)nLen;
    return "ok";

}

std::string RtLed::RtLedDisplay(int ledColour)
{
    uint8_t nLen = 0;
    uint8_t u8DisplayData[21] = {0xDD, 0x55, 0xEE, 0x00, 0x00, 0x00, 0x01, 0x00, 0x99, 
        0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x000, 0x00, 0xAA, 0xBB};
    // uint8_t u8RecvData[255] = {0};

    if (fd < 0)
    {
        return "error";
    }

    u8DisplayData[16] = ledColour >> 16;
    u8DisplayData[17] = ledColour >> 8;
    u8DisplayData[18] = ledColour;

    nLen = write(fd, (char *)u8DisplayData, sizeof(u8DisplayData));
    // nLen = read(fd, (char *)u8RecvData, sizeof(u8RecvData));
    (void)nLen;
    return "ok";
}