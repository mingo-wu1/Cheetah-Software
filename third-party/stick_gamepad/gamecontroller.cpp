/*! @file GameController.cpp
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 *
 *  NOTE: Because QT is weird, the updateDriverCommand has to be called from a
 * QT event. Running it in another thread will cause it to not work. As a
 * result, this only works if called in the update method of a QTObject
 */

#include "gamecontroller.h"

#include <QtCore/QObject>
#include <QDebug>
#include <QtGamepad/QGamepad>
#include <iostream>

#define MAX_GEAR    (22)

/*!
 * By default, the game controller selects the "first" joystick, printing a
 * warning if there are multiple joysticks On Linux, this is /dev/input/js0 If
 * no joystick is found, it will print an error message, and will return zero.
 * It is possible to change/add a joystick later with findNewController
 */
GameController::GameController(QObject *parent) : QObject(parent)
{
  findNewController();
}

GameController::~GameController() { delete _qGamepad; }

/*!
 * Re-run the joystick finding code to select the "first" joystick. This can be
 * used to set up the joystick if the simulator is started without a joystick
 * plugged in
 */
void GameController::findNewController()
{
  delete _qGamepad;
  _qGamepad = nullptr; // in case this doesn't work!

  printf("[Gamepad] Searching for gamepads, please ignore \"Device discovery cannot open device\" errors\n");
  auto gamepadList = QGamepadManager::instance()->connectedGamepads();
  printf("[Gamepad] Done searching for gamepads.\n");
  if (gamepadList.empty())
  {
    printf(
        "[ERROR: GameController] No controller was connected! All joystick "
        "commands will be zero!\n");
  }
  else
  {
    if (gamepadList.size() > 1)
    {
      printf(
          "[ERROR: GameController] There are %d joysticks connected.  Using "
          "the first one.\n",
          gamepadList.size());
    }
    else
    {
      printf("[GameController] Found 1 joystick\n");
    }

    _qGamepad = new QGamepad(*gamepadList.begin());
    qDebug() << _qGamepad->name();
    qDebug() << _qGamepad;
  }
}

/*!
 * Overwrite a driverCommand with the current joystick state.  If there's no
 * joystick, sends zeros
 * TODO: what happens if the joystick is unplugged?
 */
void GameController::updateGamepadCommand(gamepad_lcmt &gamepadCommand)
{

  static int gear = 1;
  static int rc_mode = 12;

  if (_qGamepad)
  {
    gamepadCommand.leftBumper = _qGamepad->buttonL1();
    gamepadCommand.rightBumper = _qGamepad->buttonR1();
    gamepadCommand.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    gamepadCommand.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    gamepadCommand.back = _qGamepad->buttonSelect();
    gamepadCommand.start = _qGamepad->buttonStart();
    gamepadCommand.a = _qGamepad->buttonA();
    gamepadCommand.b = _qGamepad->buttonB();
    gamepadCommand.x = _qGamepad->buttonX();
    gamepadCommand.y = _qGamepad->buttonY();
    gamepadCommand.leftStickButton = _qGamepad->buttonL3();
    gamepadCommand.rightStickButton = _qGamepad->buttonR3();
    gamepadCommand.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    gamepadCommand.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    gamepadCommand.buttonUp = _qGamepad->buttonUp();
    gamepadCommand.buttonDown = _qGamepad->buttonDown();
    gamepadCommand.buttonLeft = _qGamepad->buttonLeft();
    gamepadCommand.buttonRight = _qGamepad->buttonRight();

    if(_qGamepad->buttonUp())
      gear+=1;
    else if(_qGamepad->buttonDown())
      gear-=1;
    else{}
    gear = gear >= MAX_GEAR ? MAX_GEAR : (gear <= 0 ? 0 : gear);
    if(_qGamepad->buttonUp() || _qGamepad->buttonDown())std::cout<<"gear:"<<gear<<std::endl;

    if(gamepadCommand.leftBumper)
      rc_mode = 12;
    else if(gamepadCommand.x)
      rc_mode = 3;
    /// Add Begin by peibo 2021-04-29,add stair operation
    else if (gamepadCommand.back)
		  rc_mode = 15;
    /// Add End
    else if(gamepadCommand.b)
      gear = 0;
    else{}

    /// Mod Begin by peibo 2021-04-29,add stair operation
    if (12 == rc_mode) {
      gamepadCommand.leftStickAnalog[0] = _qGamepad->axisLeftX() * 0.5;
      gamepadCommand.leftStickAnalog[1] = -_qGamepad->axisLeftY() * 0.1 * gear;
      gamepadCommand.rightStickAnalog[0] = _qGamepad->axisRightX() * 0.8;
      gamepadCommand.rightStickAnalog[1] = -_qGamepad->axisRightY() * 0.8;
    }
    else if (3 == rc_mode) {
    	gamepadCommand.leftStickAnalog[0] = _qGamepad->axisLeftX();
    	gamepadCommand.leftStickAnalog[1] = 0;
    	gamepadCommand.rightStickAnalog[1] = _qGamepad->axisRightX() * 0.5;
    	gamepadCommand.rightStickAnalog[0] = -_qGamepad->axisRightY() * 0.4;
    }
    else if (15 == rc_mode) {
    	gamepadCommand.leftStickAnalog[0] = _qGamepad->axisLeftX() * 0.5;
    	gamepadCommand.leftStickAnalog[1] = -_qGamepad->axisLeftY() * 0.1 * gear;
    	gamepadCommand.rightStickAnalog[0] = _qGamepad->axisRightX() * 0.2;
    	gamepadCommand.rightStickAnalog[1] = -_qGamepad->axisRightY() * 0.2;
    }
    else{}

    /// Ori Code:
    // if(12 == rc_mode){
    //   gamepadCommand.leftStickAnalog[0] = _qGamepad->axisLeftX() * 0.15;
    //   gamepadCommand.leftStickAnalog[1] = -_qGamepad->axisLeftY() * 0.1 * gear;
    //   gamepadCommand.rightStickAnalog[0] = _qGamepad->axisRightX() * 0.5;
    //   gamepadCommand.rightStickAnalog[1] = -_qGamepad->axisRightY() * 0.5;
    // }
    // else if(3 == rc_mode){
    //   gamepadCommand.leftStickAnalog[0] = _qGamepad->axisLeftX();
    //   gamepadCommand.leftStickAnalog[1] = 0;
    //   gamepadCommand.rightStickAnalog[1] = _qGamepad->axisRightX() * 0.5;
    //   gamepadCommand.rightStickAnalog[0] = -_qGamepad->axisRightY() * 0.4;
    // }
    // else{}
    /// Mod End
/*
    if(gamepadCommand.leftStickAnalog[0]>=0.00001 || gamepadCommand.leftStickAnalog[0]<=-0.00001)
      std::cout<<"gamepadCommand.axisLeftX:"<<gamepadCommand.leftStickAnalog[0]<<std::endl;
    if(gamepadCommand.leftStickAnalog[1]>=0.00001 || gamepadCommand.leftStickAnalog[1]<=-0.00001)
      std::cout<<"gamepadCommand.axisLeftY:"<<gamepadCommand.leftStickAnalog[1]<<std::endl;
    if(gamepadCommand.rightStickAnalog[1]>=0.00001 || gamepadCommand.rightStickAnalog[1]<=-0.00001)
      std::cout<<"gamepadCommand.axisRightX:"<<gamepadCommand.rightStickAnalog[1]<<std::endl;
    if(gamepadCommand.rightStickAnalog[0]>=0.00001 || gamepadCommand.rightStickAnalog[0]<=-0.00001)
      std::cout<<"gamepadCommand.axisRightY:"<<gamepadCommand.rightStickAnalog[0]<<std::endl;
    if(gamepadCommand.buttonUp)
      std::cout<<"gamepadCommand.buttonUp:"<<gamepadCommand.buttonUp<<std::endl;
    if(gamepadCommand.buttonDown)
      std::cout<<"gamepadCommand.buttonDown:"<<gamepadCommand.buttonDown<<std::endl;
    if(gamepadCommand.buttonLeft)
      std::cout<<"gamepadCommand.buttonLeft:"<<gamepadCommand.buttonLeft<<std::endl;
    if(gamepadCommand.buttonRight)
      std::cout<<"gamepadCommand.buttonRight:"<<gamepadCommand.buttonRight<<std::endl;
    if(gamepadCommand.leftBumper)
      std::cout<<"gamepadCommand.leftBumper:"<<gamepadCommand.leftBumper<<std::endl;
    if(gamepadCommand.rightBumper)
      std::cout<<"gamepadCommand.rightBumper:"<<gamepadCommand.rightBumper<<std::endl;
    if(gamepadCommand.leftTriggerButton)
      std::cout<<"gamepadCommand.leftTriggerButton:"<<gamepadCommand.leftTriggerButton<<std::endl;
    if(gamepadCommand.rightTriggerButton)
      std::cout<<"gamepadCommand.rightTriggerButton:"<<gamepadCommand.rightTriggerButton<<std::endl;
    if(gamepadCommand.back)
      std::cout<<"gamepadCommand.back:"<<gamepadCommand.back<<std::endl;
    if(gamepadCommand.start)
      std::cout<<"gamepadCommand.start:"<<gamepadCommand.start<<std::endl;
    if(gamepadCommand.a)
      std::cout<<"gamepadCommand.a:"<<gamepadCommand.a<<std::endl;
    if(gamepadCommand.b)
      std::cout<<"gamepadCommand.b:"<<gamepadCommand.b<<std::endl;
    if(gamepadCommand.x)
      std::cout<<"gamepadCommand.x:"<<gamepadCommand.x<<std::endl;
    if(gamepadCommand.y)
      std::cout<<"gamepadCommand.y:"<<gamepadCommand.y<<std::endl;
    if(gamepadCommand.leftStickButton)
      std::cout<<"gamepadCommand.leftStickButton:"<<gamepadCommand.leftStickButton<<std::endl;
    if(gamepadCommand.rightStickButton)
      std::cout<<"gamepadCommand.rightStickButton:"<<gamepadCommand.rightStickButton<<std::endl;
    if(gamepadCommand.leftTriggerAnalog)
      std::cout<<"gamepadCommand.leftTriggerAnalog:"<<gamepadCommand.leftTriggerAnalog<<std::endl;
    if(gamepadCommand.rightTriggerAnalog)
      std::cout<<"gamepadCommand.rightTriggerAnalog:"<<gamepadCommand.rightTriggerAnalog<<std::endl;
*/
  }
  else
  {
  }

  // printf("%s\n", gamepadCommand.toString().c_str());
}
