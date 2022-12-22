#include "lcm/lcm-cpp.hpp"
#include "gamepad_lcmt.hpp"
#include <iostream>

int main(int argc, char** argv)
{

  lcm::LCM lcm;
  if (!lcm.good())
  {
    return 1;
  }
    gamepad_lcmt _gamepad_lcmt;
    gamepad_lcmt* lcmt = &_gamepad_lcmt;
    lcmt->leftBumper = 0;
    lcmt->rightBumper = 1;
    lcmt->leftTriggerButton = 1;
    lcmt->rightTriggerButton = 1;
    lcmt->back = 0;
    lcmt->start = 0;
    lcmt->a = 0;
    lcmt->x = 0;
    lcmt->b = 0;
    lcmt->y = 0;
    lcmt->leftStickButton = 0;
    lcmt->rightStickButton = 0;
    lcmt->leftTriggerAnalog = 0;
    lcmt->rightTriggerAnalog = 0;
    for (int i = 0; i < 2; i++) {
      lcmt->leftStickAnalog[i] = 0;
      lcmt->rightStickAnalog[i] = 0;
    }

  lcm.publish("interface", &_gamepad_lcmt);
  std::cout << "send success!"<<std::endl;
}