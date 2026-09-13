//-----------------------------------------------------------------------------
// CSimulation.h
//
// Coordinates the wall map, wall-following robot, renderer, completion
// condition, and console reporting for the A1 simulation.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRobot.h"

class CSimulation
{
    public:
        // Runs until the robot completes one lap or the window is closed.
        // Returns zero only when the lap is completed.
        int Run();

    private:
        // Draws every segment of a closed wall loop.
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop ) const;

        static const int StepsPerFrame;
};

#endif
