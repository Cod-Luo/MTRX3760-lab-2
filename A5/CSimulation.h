//-----------------------------------------------------------------------------
// CSimulation.h
//
// Declares the A5 experiment that owns both maps and runs twenty noisy wall
// followers and twenty noisy line followers simultaneously.
//-----------------------------------------------------------------------------
#ifndef CSIMULATION_H
#define CSIMULATION_H
#include "CLoopReader.h"
#include "CRobot.h"
#include "CLineRobot.h"

class CSimulation
{
    public:
        // Loads both courses, runs fixed simulation updates, retains the final
        // trajectories on screen, and reports how many robots completed a lap.
        int Run();
    private:
        //---Simulation setup and fixed-step updates---
        bool LoadMaps();
        void CreateRobots();
        void Advance();
        int CountCompleted( bool aWall ) const;
        bool AllCompleted() const;

        //---Rendering and reporting---
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop, float aThickness ) const;
        void Draw( CRender& aRender ) const;
        void PrintSummary( int aUpdates ) const;

        // Twenty of each type is required; the limit prevents an endless run
        // if unusually large accumulated noise causes a robot to lose course.
        static const int RobotsPerType = 20;
        static const int MaximumUpdates = 12000;

        //---Owned maps and robot populations---
        CLoopReader mWalls;
        CLoopReader mLine;
        std::vector<CRobot> mWallRobots;
        std::vector<CLineRobot> mLineRobots;
};
#endif
