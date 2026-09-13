//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the simultaneous A5 population experiment, including map setup,
// fixed-step updates, distinct trajectory rendering and completion reporting.
//-----------------------------------------------------------------------------
#include "CSimulation.h"
#include <iostream>

bool CSimulation::LoadMaps()
{
    const bool Okay = mWalls.ReadFile( "SimpleWalls.map" )
                    && mLine.ReadFile( "SimpleLine.map" )
                    && mWalls.GetVertices().size() >= 3 && mLine.GetVertices().size() >= 3;
    if( !Okay )
    {
        std::cerr << "Could not load both simulation maps.\n";
    }
    return Okay;
}

void CSimulation::CreateRobots()
{
    // Constructing each robot samples a new starting pose. Reserving first
    // avoids reallocating and copying the growing populations.
    mWallRobots.clear();
    mLineRobots.clear();
    mWallRobots.reserve( RobotsPerType );
    mLineRobots.reserve( RobotsPerType );
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots.push_back( CRobot( mWalls.GetStartPose().mPosition,
                                      mWalls.GetStartPose().mHeading ) );
        mLineRobots.push_back( CLineRobot( mLine.GetStartPose().mPosition,
                                          mLine.GetStartPose().mHeading ) );
    }
}

void CSimulation::Advance()
{
    // Every active robot receives exactly one equal-duration simulation step.
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots[i].Update( mWalls.GetVertices() );
        mLineRobots[i].Update( mLine.GetVertices() );
    }
}

int CSimulation::CountCompleted( bool aWall ) const
{
    // aWall selects the population while keeping the counting rule in one place.
    int Count = 0;
    for( int i = 0; i < RobotsPerType; ++i )
    {
        const bool Completed = aWall ? mWallRobots[i].HasCompletedLap()
                                     : mLineRobots[i].HasCompletedLap();
        if( Completed )
        {
            ++Count;
        }
    }
    return Count;
}

bool CSimulation::AllCompleted() const
{
    return CountCompleted( true ) == RobotsPerType && CountCompleted( false ) == RobotsPerType;
}

void CSimulation::DrawLoop( CRender& aRender, const CLoopReader& aLoop, float aThickness ) const
{
    const CRender::Colour MapColour = { 245, 245, 245, 255 };
    const std::vector<Vec2D>& Vertices = aLoop.GetVertices();
    for( std::size_t i = 0; i < Vertices.size(); ++i )
    {
        aRender.DrawLine( Vertices[i], Vertices[(i + 1) % Vertices.size()], aThickness, MapColour );
    }
}

void CSimulation::Draw( CRender& aRender ) const
{
    // Every robot has a distinct trail colour so nearby noisy trajectories can
    // still be distinguished in the required end-of-run screenshot.
    const int TrailColourCount = 20;
    const CRender::Colour WallTrailColours[TrailColourCount] = {
        { 255, 50, 50, 255 },   { 255, 120, 30, 255 },  { 255, 200, 30, 255 },
        { 210, 255, 40, 255 },  { 90, 240, 60, 255 },   { 20, 220, 130, 255 },
        { 20, 220, 220, 255 },  { 40, 150, 255, 255 },  { 70, 80, 255, 255 },
        { 150, 70, 255, 255 },  { 230, 60, 240, 255 },  { 255, 60, 160, 255 },
        { 255, 130, 130, 255 }, { 255, 175, 90, 255 },  { 245, 235, 100, 255 },
        { 150, 255, 110, 255 }, { 90, 245, 190, 255 },  { 100, 210, 255, 255 },
        { 140, 140, 255, 255 }, { 235, 130, 255, 255 }
    };
    const CRender::Colour LineTrailColours[TrailColourCount] = {
        { 255, 35, 35, 255 },   { 30, 110, 255, 255 },  { 220, 40, 220, 255 },
        { 20, 210, 80, 255 },   { 135, 45, 235, 255 },  { 0, 190, 210, 255 },
        { 255, 55, 135, 255 },  { 45, 65, 210, 255 },   { 0, 155, 125, 255 },
        { 190, 30, 75, 255 },   { 85, 190, 255, 255 },  { 175, 75, 255, 255 },
        { 60, 235, 155, 255 },  { 245, 70, 200, 255 },  { 25, 135, 175, 255 },
        { 230, 85, 95, 255 },   { 75, 100, 235, 255 },  { 110, 210, 65, 255 },
        { 165, 55, 145, 255 },  { 35, 220, 220, 255 }
    };
    const CRender::Colour Red = { 230, 41, 55, 255 };
    const CRender::Colour Yellow = { 253, 249, 0, 255 };
    const CRender::Colour SkyBlue = { 102, 191, 255, 255 };
    const CRender::Colour Blue = { 0, 121, 241, 255 };
    aRender.BeginDrawing();
    DrawLoop( aRender, mWalls, 2.0f );
    DrawLoop( aRender, mLine, 5.0f );
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots[i].Draw( aRender, WallTrailColours[i], Red, Yellow );
        mLineRobots[i].Draw( aRender, LineTrailColours[i], SkyBlue, Blue );
    }
    aRender.EndDrawing();
}

int CSimulation::Run()
{
    int Result = 1;
    if( LoadMaps() )
    {
        CreateRobots();
        int Updates = 0;
        CRender Render;
        while( !Render.WindowShouldClose() )
        {
            // Movement stops after completion or the safety limit, but drawing
            // continues so the full set of trajectories remains visible.
            if( !AllCompleted() && Updates < MaximumUpdates )
            {
                Advance();
                ++Updates;
            }
            Draw( Render );
        }
        // A5 requires most of each type, and does not grade collision counts.
        if( CountCompleted( true ) > RobotsPerType / 2
            && CountCompleted( false ) > RobotsPerType / 2 )
        {
            Result = 0;
        }
    }
    return Result;
}
