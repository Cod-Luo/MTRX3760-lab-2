//-----------------------------------------------------------------------------
// CLineRobot.cpp
//
// Implements the line-following robot. Its two binary sensor readings directly
// control its two wheels; map vertices and predetermined turns are never used
// for steering.
//-----------------------------------------------------------------------------

#include "CLineRobot.h"
#include <cmath>
#include <iostream>

const float CLineRobot::Radius = 15.0f;

//-----------------------------------------------------------------------------
CLineRobot::CLineRobot( const Vec2D& aStartPosition, float aStartHeading )
    : mPosition( aStartPosition ), mHeading( aStartHeading ), mStartPosition( aStartPosition ),
      mCentreSensor( 4.0f, 0.0f ), mSideSensor( 4.0f, 5.0f ), mTurnRight( false ),
      mLeftSpeed( 0.0f ), mRightSpeed( 0.0f ), mDistanceTravelled( 0.0f ),
      mTrail(), mUpdates( 0 ), mCollisions( 0 ), mWasColliding( false ),
      mLeftStart( false ), mCompleted( false )
{
    // Both sensors sit ahead of the centre so corners are detected earlier.
    // The second sensor is five units to the right to distinguish right turns.
    mTrail.push_back( mPosition );
}

//-----------------------------------------------------------------------------
bool CLineRobot::HasCompletedLap() const
{
    return mCompleted;
}

//-----------------------------------------------------------------------------
// The centre sensor means the robot is correctly placed and can drive ahead.
// Otherwise, the side reading selects which direction recovers the line.
//-----------------------------------------------------------------------------
void CLineRobot::Steer( const std::vector<Vec2D>& aLine )
{
    const float Speed = 30.0f;
    const float Turn = 45.0f;
    const float CornerSpeed = 27.0f;
    const bool Centre = mCentreSensor.Read( mPosition, mHeading, aLine );
    const bool Side = mSideSensor.Read( mPosition, mHeading, aLine );
    float Correction = 0.0f;
    if( Side )
    {
        mTurnRight = true;
        Correction = Turn;
    }
    else if( !Centre )
    {
        // Continue the previous right recovery across a brief gap at a corner.
        Correction = mTurnRight ? Turn : -Turn;
    }
    else
    {
        mTurnRight = false;
    }
    const float ForwardSpeed = Correction == 0.0f ? Speed : CornerSpeed;
    mLeftSpeed = ForwardSpeed + Correction;
    mRightSpeed = ForwardSpeed - Correction;
}

//-----------------------------------------------------------------------------
void CLineRobot::Update( const std::vector<Vec2D>& aLine, const std::vector<Vec2D>& aWalls )
{
    if( !mCompleted )
    {
        const float TimeStep = 0.03f;
        const float WheelBase = 2.0f * Radius;
        const float TrailSpacing = 1.0f;
        Steer( aLine );
        const float Speed = (mLeftSpeed + mRightSpeed) / 2.0f;
        const float TurnRate = (mLeftSpeed - mRightSpeed) / WheelBase;
        // Screen headings increase clockwise; wheel-speed difference sets turning.
        mHeading += TurnRate * TimeStep;
        mPosition.x += Speed * std::cos( mHeading ) * TimeStep;
        mPosition.y += Speed * std::sin( mHeading ) * TimeStep;
        mDistanceTravelled += std::fabs( Speed ) * TimeStep;
        ++mUpdates;
        CheckCollision( aWalls );
        CheckLap();
        const float TrailDx = mPosition.x - mTrail.back().x;
        const float TrailDy = mPosition.y - mTrail.back().y;
        if( TrailDx * TrailDx + TrailDy * TrailDy >= TrailSpacing * TrailSpacing || mCompleted )
        {
            mTrail.push_back( mPosition );
        }
    }
}

//-----------------------------------------------------------------------------
void CLineRobot::CheckCollision( const std::vector<Vec2D>& aWalls )
{
    // Count the start of a contact once, rather than counting every update for
    // which the robot remains in contact with the same wall.
    const bool Colliding = CLineSensor::DistanceTo( mPosition, aWalls ) <= Radius;
    if( Colliding && !mWasColliding )
    {
        ++mCollisions;
        std::cout << "Line collision at update " << mUpdates << "; total " << mCollisions << '\n';
    }
    mWasColliding = Colliding;
}

//-----------------------------------------------------------------------------
void CLineRobot::CheckLap()
{
    const float LeaveDistance = 100.0f;
    const float ReturnDistance = 5.0f;
    const float MinimumTravel = 500.0f;
    const float Dx = mPosition.x - mStartPosition.x;
    const float Dy = mPosition.y - mStartPosition.y;
    const float Distance = std::sqrt( Dx * Dx + Dy * Dy );

    // Requiring the robot to leave the start and travel a minimum distance
    // prevents small movements around the initial pose from completing a lap.
    if( Distance > LeaveDistance )
    {
        mLeftStart = true;
    }
    if( mLeftStart && mDistanceTravelled > MinimumTravel && Distance < ReturnDistance )
    {
        mCompleted = true;
        std::cout << "Line lap completed at update " << mUpdates << '\n';
    }
}

//-----------------------------------------------------------------------------
void CLineRobot::Draw( CRender& aRender ) const
{
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        aRender.DrawLine( mTrail[i - 1], mTrail[i], 1.0f, GREEN );
    }
    aRender.DrawLine( mTrail.back(), mPosition, 1.0f, GREEN );
    aRender.DrawCircle( mPosition, int( Radius ), SKYBLUE );
    const Vec2D HeadingEnd = { mPosition.x + Radius * std::cos( mHeading ),
                               mPosition.y + Radius * std::sin( mHeading ) };
    aRender.DrawLine( mPosition, HeadingEnd, 2.0f, BLUE );
}

//-----------------------------------------------------------------------------
void CLineRobot::PrintSummary() const
{
    std::cout << "Line follower: updates=" << mUpdates << ", collisions=" << mCollisions
              << ", lap=" << (mCompleted ? "complete" : "incomplete") << '\n';
}
