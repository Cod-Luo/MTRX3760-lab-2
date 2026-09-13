// TestCWallRobot.cpp - Entry point for the A1 robot simulation.
// Build: g++ -std=c++11 -Wall -Wextra TestCWallRobot.cpp CSimulation.cpp CWallRobot.cpp CSensor.cpp CLoopReader.cpp CRender.cpp -lraylib -o TestCWallRobot
// Run from A1 so the map files can be found.

#include "CSimulation.h"

int main()
{
    CSimulation Simulation;
    return Simulation.Run();
}
