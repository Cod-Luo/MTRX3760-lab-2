// Runs the A1 wall-following robot simulation: loads SimpleWalls.map,
// creates a robot at its starting pose, and lets it follow the wall on
// its right until it completes one full lap.
//
// Build (with raylib installed via apt):
//
//     g++ -Wall -Wextra TestCRobot.cpp CRobot.cpp CSensor.cpp CLoopReader.cpp CRender.cpp -lraylib -o TestCRobot

#include "CRobot.h"
#include "CLoopReader.h"
#include "CRender.h"
#include <iostream>


// Draws a loop as a closed outline: an edge between each vertex and the
// next, with the last vertex joining back to the first.

void DrawLoop( CRender& aRender, const CLoopReader& aLoop )
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

int main()
{
    CLoopReader Loop;
    Loop.ReadFile( "SimpleWalls.map" );
    CRobot Robot( Loop.GetStartPose().mPosition, Loop.GetStartPose().mHeading );
    CRender Render;

    std::cout << "Sensor 90: " << Robot.GetSensor90Distance( Loop.GetVertices() ) << std::endl;
    std::cout << "Sensor 45: " << Robot.GetSensor45Distance( Loop.GetVertices() ) << std::endl;
    
    
// Run several simulation steps per rendered frame, so the small,
// careful timestep used for accurate cornering
    const int StepsPerFrame = 60;
    
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
    Render.CloseWindow();
    std::cout << "--- Run Summary ---" << std::endl;
    std::cout << "Updates completed: " << Robot.GetUpdateCount() << std::endl;
    std::cout << "Total collisions: " << Robot.GetCollisionCount() << std::endl;

    return 0;
}


