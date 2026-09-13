//-----------------------------------------------------------------------------
// TestCRobot.cpp
//
// Entry point for A2. It starts one simulation containing the A1 wall follower
// and the A2 line follower, so both robots run in the same room at the same time.
//
// Build:
// g++ -Wall -Wextra TestCRobot.cpp CSimulation.cpp CLineRobot.cpp \
//     CLineSensor.cpp CRobot.cpp CSensor.cpp CLoopReader.cpp CRender.cpp \
//     -lraylib -o TestCRobot
//-----------------------------------------------------------------------------

#include "CSimulation.h"

int main()
{
    CSimulation Simulation;
    return Simulation.Run();
}
