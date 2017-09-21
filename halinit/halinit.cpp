#include <unistd.h>
#include <iostream>
#include <thread>
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"

using namespace std;

void keepalive() {
  while (true) {
    HAL_WaitForDSData();

    HAL_ObserveUserProgramStarting();
    HAL_ObserveUserProgramDisabled();
    HAL_ObserveUserProgramAutonomous();
    HAL_ObserveUserProgramTeleop();
    HAL_ObserveUserProgramTest();

    usleep(10000);
  }
}

int main(int argc, char *argv[]) {
  cout << "Initializing HAL..." << endl;
  if (HAL_Initialize(0, 0)) {
    cout << "HAL initialized!" << endl;
  }
  else {
    cout << "HAL initialization failed" << endl;
  }

  // Must call this function in order for the driver station to display robot
  // code.
  HAL_ObserveUserProgramStarting();

  std::thread keepaliveThread(keepalive);

  // Loop forever.
  while (true) {
    usleep(10000);
  }
  keepaliveThread.join();

  return 0;
}
