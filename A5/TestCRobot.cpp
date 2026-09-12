// TestCRobot.cpp - Entry point for the A5 noisy robot simulation.
// Build: g++ -std=c++11 -Wall -Wextra *.cpp -lraylib -o TestCRobot
// On this Mac, add -I/opt/homebrew/include -L/opt/homebrew/lib.
// Run from A5 so the maps can be found. Pass --headless to skip drawing.

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
        std::cerr << "Usage: ./TestCRobot [--headless]\n";
    }
    return Result;
}
