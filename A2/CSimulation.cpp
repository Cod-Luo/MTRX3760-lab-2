//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the complete A2 run. The A1 wall-following class is used without
// changing its behaviour, while CLineRobot adds the second required robot.
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include <iostream>

//-----------------------------------------------------------------------------
bool CSimulation::LoadMaps()
{
    // A closed loop needs at least three vertices; rejecting invalid input here
    // keeps that concern out of both robot classes.
    const bool Okay = mWalls.ReadFile( "SimpleWalls.map" )
                    && mLine.ReadFile( "SimpleLine.map" )
                    && mWalls.GetVertices().size() >= 3 && mLine.GetVertices().size() >= 3;
    if( !Okay )
    {
        std::cerr << "Could not load both simulation maps.\n";
    }
    return Okay;
}

//-----------------------------------------------------------------------------
void CSimulation::Advance( CRobot& aWallRobot, CLineRobot& aLineRobot ) const
{
    // Each robot freezes at its finishing point while the other completes.
    if( !aWallRobot.HasCompletedLap() )
    {
        aWallRobot.Update( mWalls.GetVertices() );
    }
    aLineRobot.Update( mLine.GetVertices(), mWalls.GetVertices() );
}

//-----------------------------------------------------------------------------
void CSimulation::DrawLoop( CRender& aRender, const CLoopReader& aLoop, float aThickness ) const
{
    const std::vector<Vec2D>& Vertices = aLoop.GetVertices();
    for( std::size_t i = 0; i < Vertices.size(); ++i )
    {
        aRender.DrawLine( Vertices[i], Vertices[(i + 1) % Vertices.size()], aThickness, RAYWHITE );
    }
}

//-----------------------------------------------------------------------------
void CSimulation::PrintSummary( const CRobot& aWallRobot, const CLineRobot& aLineRobot ) const
{
    std::cout << "Wall follower updates: " << aWallRobot.GetUpdateCount() << '\n'
              << "Wall follower collisions: " << aWallRobot.GetCollisionCount() << '\n';
    aLineRobot.PrintSummary();
}

//-----------------------------------------------------------------------------
int CSimulation::Run()
{
    int Result = 1;
    if( LoadMaps() )
    {
        CRobot WallRobot( mWalls.GetStartPose().mPosition, mWalls.GetStartPose().mHeading );
        CLineRobot LineRobot( mLine.GetStartPose().mPosition, mLine.GetStartPose().mHeading );
        const int MaximumUpdates = 200000;
        int Updates = 0;
        bool Finished = false;
        bool SummaryPrinted = false;
        CRender Render;

        // One fixed simulation update is shown per rendered frame. Real
        // elapsed time never enters either robot's movement calculation.
        const int StepsPerFrame = 1;
        while( !Render.WindowShouldClose() )
        {
            for( int i = 0; i < StepsPerFrame && !Finished && Updates < MaximumUpdates; ++i )
            {
                Advance( WallRobot, LineRobot );
                ++Updates;
                Finished = WallRobot.HasCompletedLap() && LineRobot.HasCompletedLap();
            }
            Render.BeginDrawing();
            DrawLoop( Render, mWalls, 2.0f );
            DrawLoop( Render, mLine, 5.0f );
            WallRobot.Draw( Render );
            LineRobot.Draw( Render );
            Render.EndDrawing();
            if( !SummaryPrinted && (Finished || Updates == MaximumUpdates) )
            {
                PrintSummary( WallRobot, LineRobot );
                SummaryPrinted = true;
            }
        }
        Render.CloseWindow();
        if( !SummaryPrinted )
        {
            PrintSummary( WallRobot, LineRobot );
        }
        const int MaximumCollisions = 10;
        if( Finished && WallRobot.GetCollisionCount() <= MaximumCollisions )
        {
            Result = 0;
        }
    }
    return Result;
}
