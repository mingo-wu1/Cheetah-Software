/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "BZL_Controller.hpp"
#include "Logger/Logger.h"

int main(int argc, char** argv) {

  BZL_QUADRUPED::LoggerInit(argc, argv);

  main_helper(argc, argv, new BZL_Controller());

  BZL_QUADRUPED::LoggerShutDown();

  return 0;
}
