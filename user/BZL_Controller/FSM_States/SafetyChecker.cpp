/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation() {
  if (abs(data->_stateEstimator->getResult().rpy(0)) >= 0.5 ||
      abs(data->_stateEstimator->getResult().rpy(1)) >= 0.5) {
        QUADRUPED_ERROR(_logger, "Orientation safety check failed!");
    return false;
  } else {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot() {
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  T maxAngle = 1.0472;  // 60 degrees (should be changed)
  T maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Keep the foot from going too far from the body in +x
    if (data->_legController->commands[leg].pDes(0) > maxPDes) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 0, data->_legController->commands[leg].pDes(0), maxPDes);
      data->_legController->commands[leg].pDes(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (data->_legController->commands[leg].pDes(0) < -maxPDes) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 0, data->_legController->commands[leg].pDes(0), -maxPDes);
      data->_legController->commands[leg].pDes(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (data->_legController->commands[leg].pDes(1) > maxPDes) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 1, data->_legController->commands[leg].pDes(1), maxPDes);
      data->_legController->commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->commands[leg].pDes(1) < -maxPDes) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 1, data->_legController->commands[leg].pDes(1), -maxPDes);
      data->_legController->commands[leg].pDes(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    if (data->_legController->commands[leg].pDes(2) >
        -data->_quadruped->_maxLegLength / 4) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 2, data->_legController->commands[leg].pDes(2), -data->_quadruped->_maxLegLength / 4);
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength / 4;
      safePDesFoot = false;
    }

    // Keep the foot within the kinematic limits
    if (data->_legController->commands[leg].pDes(2) <
        -data->_quadruped->_maxLegLength) {
      QUADRUPED_INFO(_logger, "PDes leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 2, data->_legController->commands[leg].pDes(2), -data->_quadruped->_maxLegLength);
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength;
      safePDesFoot = false;
    }
  }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward() {
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  T maxLateralForce = 0;
  T maxVerticalForce = 0;

  // Maximum force limits for each robot
  if (data->_quadruped->_robotType == RobotType::CHEETAH_3) {
    maxLateralForce = 1800;
    maxVerticalForce = 1800;

  } else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    maxLateralForce = 350;
    maxVerticalForce = 350;
  }

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Limit the lateral forces in +x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) >
        maxLateralForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 0, data->_legController->commands[leg].forceFeedForward(0), maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) <
        -maxLateralForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 0, data->_legController->commands[leg].forceFeedForward(0), -maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) >
        maxLateralForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 1, data->_legController->commands[leg].forceFeedForward(1), maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) <
        -maxLateralForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 1, data->_legController->commands[leg].forceFeedForward(1), -maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) >
        maxVerticalForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 2, data->_legController->commands[leg].forceFeedForward(2), -maxVerticalForce);
      data->_legController->commands[leg].forceFeedForward(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) <
        -maxVerticalForce) {
      QUADRUPED_INFO(_logger, "Force leg: %d | coordinate: %d\n"
        "\tcommanded: %f | modified: %f",
        leg, 2, data->_legController->commands[leg].forceFeedForward(2), maxVerticalForce);
      data->_legController->commands[leg].forceFeedForward(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
