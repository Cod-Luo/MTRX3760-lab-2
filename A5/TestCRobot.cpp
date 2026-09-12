// TestCRobot.cpp - Entry point for the A5 noisy robot simulation.
// Build: g++ -std=c++11 -Wall -Wextra *.cpp -lraylib -o TestCRobot
// On this Mac, add -I/opt/homebrew/include -L/opt/homebrew/lib.
// Run from A5 so the maps can be found.

#include "CSimulation.h"

int main()
{
    CSimulation Simulation;
    return Simulation.Run();
}
