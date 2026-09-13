//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the complete A1 wall-following simulation.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>

const int CSimulation::StepsPerFrame = 1;

//-----------------------------------------------------------------------------
int CSimulation::Run()
{
    CLoopReader Loop;
    if( !Loop.ReadFile( "SimpleWalls.map" ) || Loop.GetVertices().size() < 3 )
    {
        std::cerr << "Could not load a valid wall map." << std::endl;
        return 1;
    }

    CRobot Robot( Loop.GetStartPose().mPosition, Loop.GetStartPose().mHeading );
    CRender Render;

    std::cout << "Wall sensors: 90 degrees="
              << Robot.GetSensor90Distance( Loop.GetVertices() )
              << ", 45 degrees="
              << Robot.GetSensor45Distance( Loop.GetVertices() ) << '\n';

    bool SummaryPrinted = false;
    while( !Render.WindowShouldClose() )
    {
        for( int i = 0; i < StepsPerFrame && !Robot.HasCompletedLap(); ++i )
        {
            Robot.Update( Loop.GetVertices() );
        }

        Render.BeginDrawing();
        DrawLoop( Render, Loop );
        Robot.Draw( Render );
        Render.EndDrawing();

        if( Robot.HasCompletedLap() && !SummaryPrinted )
        {
            std::cout << "Wall follower: updates=" << Robot.GetUpdateCount()
                      << ", collisions=" << Robot.GetCollisionCount()
                      << ", lap=complete\n"
                      << "Run complete. Close the window to exit.\n";
            SummaryPrinted = true;
        }
    }

    Render.CloseWindow();
    if( !SummaryPrinted )
    {
        std::cout << "Wall follower: updates=" << Robot.GetUpdateCount()
                  << ", collisions=" << Robot.GetCollisionCount()
                  << ", lap=incomplete\n";
    }
    return Robot.HasCompletedLap() ? 0 : 1;
}

//-----------------------------------------------------------------------------
void CSimulation::DrawLoop( CRender& aRender, const CLoopReader& aLoop ) const
{
    const float EdgeThickness = 2.0f;
    const std::vector<Vec2D>& Vertices = aLoop.GetVertices();
    if( !Vertices.empty() )
    {
        Vec2D Previous = Vertices.back();
        for( const Vec2D& Vertex : Vertices )
        {
            aRender.DrawLine( Previous, Vertex, EdgeThickness, RAYWHITE );
            Previous = Vertex;
        }
    }
}
