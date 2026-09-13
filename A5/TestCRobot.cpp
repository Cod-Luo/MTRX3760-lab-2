//-----------------------------------------------------------------------------
// TestCRobot.cpp
//
// Entry point for the A5 noisy-population simulation.
// Build: g++ -std=c++11 -Wall -Wextra *.cpp -lraylib -o TestCRobot
// Run from A5 so the map files can be found.
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include <cstdlib>

int main()
{
    // A fixed seed makes the assessed population and screenshot reproducible.
    const unsigned int RandomSeed = 20;
    std::srand( RandomSeed );

    CSimulation Simulation;
    return Simulation.Run();
}
