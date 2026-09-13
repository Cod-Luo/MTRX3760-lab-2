//-----------------------------------------------------------------------------
// TestCRobot.cpp
//
// Entry point for the A1 wall-following robot simulation.
//
// Build (with raylib installed via apt):
//
//     g++ -Wall -Wextra TestCRobot.cpp CSimulation.cpp CRobot.cpp CSensor.cpp \
//         CLoopReader.cpp CRender.cpp -lraylib -o TestCRobot
//-----------------------------------------------------------------------------

#include "CSimulation.h"

//-----------------------------------------------------------------------------
int main()
{
    CSimulation Simulation;
    return Simulation.Run();
}
