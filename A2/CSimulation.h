//-----------------------------------------------------------------------------
// CSimulation.h
//
// Declares the top-level owner of both maps and coordinates the simultaneous
// wall-following and line-following robot simulation required for A2.
//-----------------------------------------------------------------------------
#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRobot.h"
#include "CLineRobot.h"

//-----------------------------------------------------------------------------
// CSimulation loads the two courses, advances both robots using a fixed
// timestep, renders them in one window, and prints their end-of-run summaries.
//-----------------------------------------------------------------------------
class CSimulation
{
    public:
        // Runs until both laps finish, the update limit is reached, or the user
        // closes the window. aHeadless skips drawing for automated verification.
        // Returns zero only when both robots finish and the wall robot remains
        // within the assignment's collision limit.
        int Run( bool aHeadless );

    private:
        // Loads and validates both map files before either robot is created.
        bool LoadMaps();

        // Advances each unfinished robot by one fixed simulation step.
        void Advance( CRobot& aWallRobot, CLineRobot& aLineRobot ) const;

        // Draws every segment of a closed map loop at the requested thickness.
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop, float aThickness ) const;

        // Reports the completion, update, and collision results of both robots.
        void PrintSummary( const CRobot& aWallRobot, const CLineRobot& aLineRobot ) const;

        // The simulation retains both maps for sensing and drawing throughout the run.
        CLoopReader mWalls; // room boundary followed by the range-sensor robot
        CLoopReader mLine;  // floor line followed by the binary-sensor robot
};

#endif
