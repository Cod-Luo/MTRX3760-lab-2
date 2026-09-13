// CSimulation.h - Owns the maps and 20 robots of each type.
#ifndef CSIMULATION_H
#define CSIMULATION_H
#include "CLoopReader.h"
#include "CRobot.h"
#include "CLineRobot.h"

class CSimulation
{
    public:
        // Runs and draws both populations until the window closes.
        int Run();
    private:
        bool LoadMaps();
        void CreateRobots();
        void Advance();
        int CountCompleted( bool aWall ) const;
        bool AllCompleted() const;
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop, float aThickness ) const;
        void Draw( CRender& aRender ) const;
        void PrintSummary( int aUpdates ) const;
        static const int RobotsPerType = 20;
        static const int MaximumUpdates = 12000;
        CLoopReader mWalls;
        CLoopReader mLine;
        std::vector<CRobot> mWallRobots;
        std::vector<CLineRobot> mLineRobots;
};
#endif
