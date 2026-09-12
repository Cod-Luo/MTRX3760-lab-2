// CSimulation.cpp - Simultaneous fixed-step population experiment for A5.
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

void CSimulation::CreateRobots( unsigned int aSeed )
{
    mWallRobots.clear();
    mLineRobots.clear();
    mWallRobots.reserve( RobotsPerType );
    mLineRobots.reserve( RobotsPerType );
    for( int i = 0; i < RobotsPerType; ++i )
    {
        // Distinct seeds give every robot its own stream; no generator is shared.
        const unsigned int WallSeed = aSeed + 2u * static_cast<unsigned int>( i );
        const unsigned int LineSeed = WallSeed + 1u;
        mWallRobots.push_back( CRobot( mWalls.GetStartPosition(),
                                      mWalls.GetStartHeading(), WallSeed ) );
        mLineRobots.push_back( CLineRobot( mLine.GetStartPosition(),
                                          mLine.GetStartHeading(), LineSeed ) );
    }
}

void CSimulation::Advance()
{
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots[i].Update( mWalls.GetVertices() );
        mLineRobots[i].Update( mLine.GetVertices() );
    }
}

int CSimulation::CountCompleted( bool aWall ) const
{
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
    // Several related colours make overlapping noisy paths distinguishable in
    // the final screenshot while retaining A2's yellow/green visual grouping.
    const int TrailColourCount = 5;
    const CRender::Colour WallTrailColours[TrailColourCount] = {
        { 253, 249, 0, 255 }, { 255, 210, 0, 255 }, { 255, 170, 0, 255 },
        { 255, 235, 90, 255 }, { 220, 255, 40, 255 }
    };
    const CRender::Colour LineTrailColours[TrailColourCount] = {
        { 0, 228, 48, 255 }, { 0, 200, 110, 255 }, { 40, 255, 90, 255 },
        { 0, 235, 180, 255 }, { 120, 255, 80, 255 }
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
        const int ColourIndex = i % TrailColourCount;
        mWallRobots[i].Draw( aRender, WallTrailColours[ColourIndex], Red, Yellow );
        mLineRobots[i].Draw( aRender, LineTrailColours[ColourIndex], SkyBlue, Blue );
    }
    aRender.EndDrawing();
}

void CSimulation::PrintSummary( int aUpdates ) const
{
    std::cout << "Simulation updates: " << aUpdates << '\n'
              << "Completed: wall " << CountCompleted( true ) << '/' << RobotsPerType
              << ", line " << CountCompleted( false ) << '/' << RobotsPerType << '\n';
}

int CSimulation::Run()
{
    int Result = 1;
    if( LoadMaps() )
    {
        const unsigned int Seed = 3760u;
        CreateRobots( Seed );
        int Updates = 0;
        bool SummaryPrinted = false;
        CRender Render;
        while( !Render.WindowShouldClose() )
        {
            if( !AllCompleted() && Updates < MaximumUpdates )
            {
                Advance();
                ++Updates;
            }
            const bool Stopped = AllCompleted() || Updates == MaximumUpdates;
            Draw( Render );
            if( Stopped && !SummaryPrinted )
            {
                PrintSummary( Updates );
                SummaryPrinted = true;
            }
        }
        if( !SummaryPrinted )
        {
            PrintSummary( Updates );
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
