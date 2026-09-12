//-----------------------------------------------------------------------------
// TestCRobot.cpp
//
// Entry point for A1. Runs the wall-following robot simulation: loads
// SimpleWalls.map, creates a robot at its starting pose, and lets it follow
// the wall on its right until it completes one full lap.
//
// Build (with raylib installed via apt):
//
//     g++ -Wall -Wextra TestCRobot.cpp CRobot.cpp CSensor.cpp CLoopReader.cpp CRender.cpp -lraylib -o TestCRobot
//-----------------------------------------------------------------------------

#include "CRobot.h"
#include "CLoopReader.h"
#include "CRender.h"

#include <iostream>

//-----------------------------------------------------------------------------
// CWallFollowerSimulation owns the map, the robot, and the renderer for the
// whole run, and reports the sensors, collisions, and final summary.
//-----------------------------------------------------------------------------
class CWallFollowerSimulation
{
    public:
        //---Runs the whole simulation until one lap completes, or the
        // window is closed. Returns 0 on success.---
        int Run();

    private:
        //---Draws a loop as a closed outline of its wall segments.---
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop ) const;

        //---Simulation stepping, per rendered frame.---
        static const int StepsPerFrame;
};

const int CWallFollowerSimulation::StepsPerFrame = 8;

//-----------------------------------------------------------------------------
int CWallFollowerSimulation::Run()
{
    CLoopReader Loop;
    bool ReadSuccess = Loop.ReadFile( "SimpleWalls.map" );
    std::cout << "Read " << ( ReadSuccess ? "successful" : "unsuccessful" ) << std::endl;

    CRobot Robot( Loop.GetStartPose().mPosition, Loop.GetStartPose().mHeading );
    CRender Render;

    std::cout << "Sensor 90: " << Robot.GetSensor90Distance( Loop.GetVertices() ) << std::endl;
    std::cout << "Sensor 45: " << Robot.GetSensor45Distance( Loop.GetVertices() ) << std::endl;

    while( !Render.WindowShouldClose() && !Robot.HasCompletedLap() )
    {
        for( int i = 0; i < StepsPerFrame; i++ )
        {
            Robot.Update( Loop.GetVertices() );
        }

        Render.BeginDrawing();
        DrawLoop( Render, Loop );
        Robot.Draw( Render );
        Render.EndDrawing();
    }


    for( int i = 0; i < 180; i++ )
    {
        Render.BeginDrawing();
        DrawLoop( Render, Loop );
        Robot.Draw( Render );
        Render.EndDrawing();
    }
    
    Render.CloseWindow();

    std::cout << "--- Run Summary ---" << std::endl;
    std::cout << "Updates completed: " << Robot.GetUpdateCount() << std::endl;
    std::cout << "Total collisions: " << Robot.GetCollisionCount() << std::endl;

    return 0;
}

//-----------------------------------------------------------------------------
void CWallFollowerSimulation::DrawLoop( CRender& aRender, const CLoopReader& aLoop ) const
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

//-----------------------------------------------------------------------------
int main()
{
    CWallFollowerSimulation Simulation;
    return Simulation.Run();
}
