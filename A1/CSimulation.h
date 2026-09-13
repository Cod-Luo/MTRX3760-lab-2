//-----------------------------------------------------------------------------
// CSimulation.h
//
// Coordinates the wall map, wall-following robot, renderer, completion
// condition, and console reporting for the A1 simulation.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CWallRobot.h"

class CSimulation
{
    public:
        // Stops movement after one lap and displays the trail until the window closes.
        // Returns zero for a completed lap with at most ten collisions.
        int Run();

    private:
        // Draws every segment of a closed wall loop.
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop ) const;

        static const int StepsPerFrame;
};

#endif
