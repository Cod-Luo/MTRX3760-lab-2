//-----------------------------------------------------------------------------
// TestCRobot.cpp
//
// Entry point for A2. It starts one simulation containing the A1 wall follower
// and the A2 line follower, so both robots run in the same room at the same time.
// Pass --headless to run the same simulation without opening a window.
//
// Build:
// g++ -Wall -Wextra TestCRobot.cpp CSimulation.cpp CLineRobot.cpp \
//     CLineSensor.cpp CRobot.cpp CSensor.cpp CLoopReader.cpp CRender.cpp \
//     -lraylib -o TestCRobot
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include <string>
#include <iostream>

int main( int argc, char* argv[] )
{
    int Result = 1;
    const bool Headless = argc == 2 && std::string( argv[1] ) == "--headless";
    if( argc == 1 || Headless )
    {
        CSimulation Simulation;
        Result = Simulation.Run( Headless );
    }
    else
    {
        std::cerr << "Usage: ./A2 [--headless]\n";
    }
    return Result;
}
