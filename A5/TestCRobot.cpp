// TestCRobot.cpp - Entry point for the A5 robot simulation.
// Build: g++ -std=c++11 -Wall -Wextra *.cpp -lraylib -o TestCRobot
// Run from A5 so the map files can be found.

#include "CSimulation.h"

int main()
{
    CSimulation Simulation;
    return Simulation.Run();
}
