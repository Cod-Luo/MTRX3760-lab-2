// CSimulation.cpp - Simultaneous fixed-step population experiment for A5.
#include "CSimulation.h"
#include <iostream>

bool CSimulation::LoadMaps()
{
    const bool Okay = mWalls.ReadFile( "SimpleWalls.map" )
                    && mLine.ReadFile( "SimpleLine.map" )
                    && mWalls.GetVertices().size() >= 3 && mLine.GetVertices().size() >= 3;
    if( !Okay ) { std::cerr << "Could not load both simulation maps.\n"; }
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
        mWallRobots.push_back( CRobot( mWalls.GetStartPose().mPosition,
                                     mWalls.GetStartPose().mHeading, WallSeed ) );
        mLineRobots.push_back( CLineRobot( mLine.GetStartPose().mPosition,
                                         mLine.GetStartPose().mHeading, LineSeed ) );
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
        const CNoisyMotion& Motion = aWall ? mWallRobots[i].GetMotion() : mLineRobots[i].GetMotion();
        if( Motion.HasCompletedLap() ) { ++Count; }
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
    // Use the same trail, body and heading colours as A1 and A2.
    const CRender::Colour Yellow = { 253, 249, 0, 255 };
    const CRender::Colour Green = { 0, 228, 48, 255 };
    const CRender::Colour Red = { 230, 41, 55, 255 };
    const CRender::Colour SkyBlue = { 102, 191, 255, 255 };
    const CRender::Colour Blue = { 0, 121, 241, 255 };
    aRender.BeginDrawing();
    DrawLoop( aRender, mWalls, 2.0f );
    DrawLoop( aRender, mLine, 5.0f );
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots[i].GetMotion().DrawTrail( aRender, Yellow );
        mLineRobots[i].GetMotion().DrawTrail( aRender, Green );
    }
    for( int i = 0; i < RobotsPerType; ++i )
    {
        mWallRobots[i].GetMotion().DrawBody( aRender, Red, Red );
        mLineRobots[i].GetMotion().DrawBody( aRender, SkyBlue, Blue );
    }
    aRender.EndDrawing();
}

void CSimulation::PrintSummary( int aUpdates, unsigned int aSeed ) const
{
    std::cout << "Seed: " << aSeed << "; simulation updates: " << aUpdates << '\n';
    for( int i = 0; i < RobotsPerType; ++i )
    {
        const CNoisyMotion& Wall = mWallRobots[i].GetMotion();
        const CNoisyMotion& Line = mLineRobots[i].GetMotion();
        std::cout << "Robot " << i + 1 << ": wall=" << (Wall.HasCompletedLap() ? "complete" : "incomplete")
                  << " (" << Wall.GetUpdateCount() << "), line="
                  << (Line.HasCompletedLap() ? "complete" : "incomplete")
                  << " (" << Line.GetUpdateCount() << ")\n";
    }
    std::cout << "Completed: wall " << CountCompleted( true ) << "/20, line "
              << CountCompleted( false ) << "/20\n";
}

int CSimulation::Run( bool aHeadless )
{
    int Result = 1;
    if( LoadMaps() )
    {
        const unsigned int Seed = 3760u;
        CreateRobots( Seed );
        int Updates = 0;
        if( aHeadless )
        {
            while( !AllCompleted() && Updates < MaximumUpdates )
            {
                Advance();
                ++Updates;
            }
        }
        else
        {
            CRender Render;
            const int StepsPerFrame = 1;
            bool Reported = false;
            while( !Render.WindowShouldClose() )
            {
                for( int i = 0; i < StepsPerFrame && !AllCompleted() && Updates < MaximumUpdates; ++i )
                {
                    Advance();
                    ++Updates;
                }
                const bool Stopped = AllCompleted() || Updates == MaximumUpdates;
                Draw( Render );
                if( Stopped && !Reported )
                {
                    PrintSummary( Updates, Seed );
                    Reported = true;
                }
            }
        }
        PrintSummary( Updates, Seed );
        // A5 requires most of each type, and does not grade collision counts.
        if( CountCompleted( true ) > RobotsPerType / 2
            && CountCompleted( false ) > RobotsPerType / 2 )
        {
            Result = 0;
        }
    }
    return Result;
}
