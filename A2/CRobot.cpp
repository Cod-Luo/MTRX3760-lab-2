//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Implements the A2 wall follower. The two range readings are converted into
// differential wheel speeds, then fixed-timestep motion updates the robot.
//-----------------------------------------------------------------------------

#include "CRobot.h"

#include <cmath>
#include <iostream>

const float CRobot::Radius = 15.0f;

//-----------------------------------------------------------------------------
CRobot::CRobot( const Vec2D& aStartPos, float aStartHeading )
    : mPosition( aStartPos )
    , mHeading( aStartHeading )
    , mSensor90( 90.0f )
    , mSensor45( 45.0f )
    , mLeftWheelSpeed( 1.0f )
    , mRightWheelSpeed( 1.0f )
    , mCollisionCount( 0 )
    , mUpdateCount( 0 )
    , mWasColliding( false )
    , mStartPosition( aStartPos )
    , mHasLeftStart( false )
    , mLapCompleted( false )
    , mDistanceTravelled( 0.0f )
{
    mTrail.push_back( mPosition );
}

//-----------------------------------------------------------------------------
void CRobot::Update( const std::vector<Vec2D>& aWalls )
{
    // Fixed simulated time keeps motion independent of rendering performance.
    const float dt = 0.03f;
    const float WheelBase = 30.0f;
    const float BaseSpeed = 42.0f;

    // The diagonal target is longer because its ray meets a parallel wall at
    // 45 degrees rather than travelling directly across the clearance.
    const float TargetSideDistance = 45.0f;
    const float TargetDiagonalDistance = TargetSideDistance * 1.41421356f;
    const float MaxSensorDistance = 120.0f;

    // Scaling both gains with BaseSpeed preserves steering authority at the
    // increased forward speed.
    const float SideGain = 2.1f;
    const float DiagonalGain = 4.2f;
    const float MaxTurn = 157.5f;

    float Dist90 = mSensor90.GetDistance( mPosition, mHeading, aWalls );
    float Dist45 = mSensor45.GetDistance( mPosition, mHeading, aWalls );

    if( Dist90 > MaxSensorDistance ) Dist90 = MaxSensorDistance;
    if( Dist45 > MaxSensorDistance ) Dist45 = MaxSensorDistance;
    

    // Steer based on both sensors together: the 90-degree sensor keeps the
    // robot at the target distance from the wall beside it, and the
    // 45-degree sensor gives an early warning of corners ahead.
    float Turn = SideGain * ( Dist90 - TargetSideDistance )
               + DiagonalGain * ( Dist45 - TargetDiagonalDistance );

    if( Turn > MaxTurn )  Turn = MaxTurn;
    if( Turn < -MaxTurn ) Turn = -MaxTurn;

    mLeftWheelSpeed  = BaseSpeed + Turn;
    mRightWheelSpeed = BaseSpeed - Turn;

    float ForwardSpeed = ( mLeftWheelSpeed + mRightWheelSpeed ) / 2.0f;
    float TurnRate = ( mLeftWheelSpeed - mRightWheelSpeed ) / WheelBase;

    mHeading += TurnRate * dt;
    mPosition.x += ForwardSpeed * std::cos( mHeading ) * dt;
    mPosition.y += ForwardSpeed * std::sin( mHeading ) * dt;
    mDistanceTravelled += std::fabs( ForwardSpeed ) * dt;

    mTrail.push_back( mPosition );
    mUpdateCount++;


    // Match A1: check the complete radius-15 disc against every wall segment.
    // Only count a new collision when contact starts.
    bool IsColliding = DistanceToNearestWall( mPosition, aWalls ) <= Radius;
    if( IsColliding && !mWasColliding )
    {
        mCollisionCount++;
        std::cout << "Collision! Total so far: " << mCollisionCount << std::endl;
    }
    mWasColliding = IsColliding;

    // A lap requires leaving, travelling approximately one circuit, and then
    // returning close to the initial centre position.
    const float LeaveThreshold = 100.0f;
    const float ReturnThreshold = 5.0f;
    const float MinimumLapDistance = 1500.0f;

    float dx = mPosition.x - mStartPosition.x;
    float dy = mPosition.y - mStartPosition.y;
    float DistanceFromStart = std::sqrt( dx * dx + dy * dy );

    if( !mHasLeftStart && DistanceFromStart > LeaveThreshold )
    {
        mHasLeftStart = true;
    }

    if( mHasLeftStart && !mLapCompleted
        && DistanceFromStart < ReturnThreshold
        && mDistanceTravelled > MinimumLapDistance )
    {
        mLapCompleted = true;
        std::cout << "Lap completed!" << std::endl;
    }
}

//-----------------------------------------------------------------------------
float CRobot::DistanceToNearestWall( const Vec2D& aPoint,
                                     const std::vector<Vec2D>& aWalls )
{
    float ClosestDistance = 999.0f;
    for( std::size_t i = 0; i < aWalls.size(); ++i )
    {
        const Vec2D& Start = aWalls[i];
        const Vec2D& End = aWalls[(i + 1) % aWalls.size()];
        const float SegmentX = End.x - Start.x;
        const float SegmentY = End.y - Start.y;
        const float ToPointX = aPoint.x - Start.x;
        const float ToPointY = aPoint.y - Start.y;
        const float SegmentLengthSquared = SegmentX * SegmentX + SegmentY * SegmentY;

        float Fraction = 0.0f;
        if( SegmentLengthSquared > 0.0f )
        {
            Fraction = (ToPointX * SegmentX + ToPointY * SegmentY)
                     / SegmentLengthSquared;
        }
        if( Fraction < 0.0f ) Fraction = 0.0f;
        if( Fraction > 1.0f ) Fraction = 1.0f;

        const float DifferenceX = aPoint.x - (Start.x + Fraction * SegmentX);
        const float DifferenceY = aPoint.y - (Start.y + Fraction * SegmentY);
        const float Distance = std::sqrt( DifferenceX * DifferenceX
                                        + DifferenceY * DifferenceY );
        if( Distance < ClosestDistance ) ClosestDistance = Distance;
    }
    return ClosestDistance;
}

void CRobot::Draw( CRender& aRender ) const
{
    for( size_t i = 1; i < mTrail.size(); i++ )
    {
        aRender.DrawLine( mTrail[i - 1], mTrail[i], 1.0f, YELLOW );
    }

    aRender.DrawCircle( mPosition, (int)Radius, RED );

    const float HeadingLineLength = Radius * 2.0f;
    Vec2D HeadingEnd;
    HeadingEnd.x = mPosition.x + HeadingLineLength * std::cos( mHeading );
    HeadingEnd.y = mPosition.y + HeadingLineLength * std::sin( mHeading );
    aRender.DrawLine( mPosition, HeadingEnd, 2.0f, RED );
}

float CRobot::GetSensor90Distance( const std::vector<Vec2D>& aWalls ) const
{
    return mSensor90.GetDistance( mPosition, mHeading, aWalls );
}

float CRobot::GetSensor45Distance( const std::vector<Vec2D>& aWalls ) const
{
    return mSensor45.GetDistance( mPosition, mHeading, aWalls );
}

int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}

int CRobot::GetUpdateCount() const
{
    return mUpdateCount;
}

bool CRobot::HasCompletedLap() const
{
    return mLapCompleted;
}
