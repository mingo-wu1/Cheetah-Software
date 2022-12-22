#ifndef PROJECT_GAMECONTROLLER_H
#define PROJECT_GAMECONTROLLER_H

#include "lcm/lcm-cpp.hpp"
#include "gamepad_lcmt.hpp"
#include <QtCore/QObject>

class QGamepad;  // for an unknown reason, #including <QtGamepad/QGamepad> here
                 // makes compilation *very* slow

class GameController : public QObject {
  Q_OBJECT
 public:
  explicit GameController(QObject *parent = 0);
  void updateGamepadCommand(gamepad_lcmt &gamepadCommand);
  void findNewController();
  ~GameController();

 private:
  QGamepad *_qGamepad = nullptr;
};

#endif  // PROJECT_GAMECONTROLLER_H

