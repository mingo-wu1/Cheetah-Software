#include <bzl_quadruped_sdk/bzl_quadruped.h>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace BZL_QUADRUPED_SDK;

int main(void)
{
    std::shared_ptr<BzlQuadruped> pBzlQuadruped = std::make_shared<BzlQuadruped>();
    
    GamepadCmd gamepadCmd;
    memset(&gamepadCmd, 0, sizeof(gamepadCmd));

    gamepadCmd.b = 1;                       // 站立
    pBzlQuadruped->SendCmd(gamepadCmd);
    gamepadCmd.b = 0;
    sleep(5);

    gamepadCmd.leftBumper = 1;              // 踏步
    pBzlQuadruped->SendCmd(gamepadCmd);
    gamepadCmd.leftBumper = 0;
    sleep(5);

    gamepadCmd.b = 1;                       // 站立
    pBzlQuadruped->SendCmd(gamepadCmd);
    gamepadCmd.b = 0;
    sleep(5);

    gamepadCmd.a = 1;                       // 趴下
    pBzlQuadruped->SendCmd(gamepadCmd);
    gamepadCmd.a = 0;
    sleep(5);

    gamepadCmd.rightBumper = 1;             // 禁能
    pBzlQuadruped->SendCmd(gamepadCmd);
    gamepadCmd.rightBumper = 0;
    sleep(1);

    return 0;
}