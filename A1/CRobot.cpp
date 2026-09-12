//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Implements the wall-following robot. Its two range sensor readings steer
// the wheels; collisions are checked against the robot's whole body, not
// just its two sensor rays.
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
    // Every call advances the same amount of simulated time, independently of
    // how long the computer takes to render a frame.
    const float dt = 0.03f;

    Steer( aWalls );

    const float WheelBase = 30.0f;
    float ForwardSpeed = ( mLeftWheelSpeed + mRightWheelSpeed ) / 2.0f;
    float TurnRate = ( mLeftWheelSpeed - mRightWheelSpeed ) / WheelBase;

    mHeading += TurnRate * dt;
    mPosition.x += ForwardSpeed * std::cos( mHeading ) * dt;
    mPosition.y += ForwardSpeed * std::sin( mHeading ) * dt;
    mDistanceTravelled += std::fabs( ForwardSpeed ) * dt;

    mTrail.push_back( mPosition );
    mUpdateCount++;

    CheckCollision( aWalls );
    CheckLap();
}

//-----------------------------------------------------------------------------
// Reads both sensors and decides the two wheel speeds. The 90-degree sensor
// keeps the robot at the target distance from the wall beside it; the
// 45-degree sensor gives an early warning of corners ahead.
//-----------------------------------------------------------------------------
void CRobot::Steer( const std::vector<Vec2D>& aWalls )
{
    const float BaseSpeed = 42.0f;
    // The side sensor holds this clearance. At 45 degrees, the diagonal ray
    // travels sqrt( 2 ) times farther to reach the same straight wall.
    const float TargetSideDistance = 45.0f;
    const float TargetDiagonalDistance = TargetSideDistance * 1.41421356f;
    const float MaxSensorDistance = 120.0f;
    // These gains are scaled with BaseSpeed so the relationship between
    // forward motion and steering remains the same as in the slower version.
    const float SideGain = 2.1f;
    const float DiagonalGain = 4.2f;
    const float MaxTurn = 157.5f;

    float Dist90 = mSensor90.GetDistance( mPosition, mHeading, aWalls );
    float Dist45 = mSensor45.GetDistance( mPosition, mHeading, aWalls );

    if( Dist90 > MaxSensorDistance ) Dist90 = MaxSensorDistance;
    if( Dist45 > MaxSensorDistance ) Dist45 = MaxSensorDistance;

    float Turn = SideGain * ( Dist90 - TargetSideDistance )
               + DiagonalGain * ( Dist45 - TargetDiagonalDistance );

    if( Turn > MaxTurn )  Turn = MaxTurn;
    if( Turn < -MaxTurn ) Turn = -MaxTurn;

    mLeftWheelSpeed  = BaseSpeed + Turn;
    mRightWheelSpeed = BaseSpeed - Turn;
}

//-----------------------------------------------------------------------------
// A collision is when the robot's whole body (its 15-unit radius disc) comes
// within Radius of any wall segment, not just where a sensor happens to
// point. Only count the start of a collision, not every update it remains
// touching the same wall.
//-----------------------------------------------------------------------------
void CRobot::CheckCollision( const std::vector<Vec2D>& aWalls )
{
    bool IsColliding = DistanceToNearestWall( mPosition, aWalls ) <= Radius;

    if( IsColliding && !mWasColliding )
    {
        mCollisionCount++;
        std::cout << "Collision! Total so far: " << mCollisionCount << std::endl;
    }
    mWasColliding = IsColliding;
}

//-----------------------------------------------------------------------------
// A lap is detected by leaving the start position by some distance, then
// returning close to it again.
//-----------------------------------------------------------------------------
void CRobot::CheckLap()
{
    const float LeaveThreshold = 100.0f;
    // As with the A2 line follower, require the robot's centre to return very
    // close to its initial position so the completed trail has no visible gap.
    const float ReturnThreshold = 5.0f;

    // Requiring a minimum distance travelled, in addition to leaving and
    // returning to the start, guards against the lap completing early if
    // the path happens to pass near the start before genuinely going all
    // the way around the room.
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
        std::cout << "Lap completed! Distance travelled: " << mDistanceTravelled << std::endl;
    }
}

//-----------------------------------------------------------------------------
// Returns the distance from aPoint to the nearest point on the wall loop
// aWalls, checking every segment. Each segment runs from one vertex to the
// next, wrapping the last back to the first.
//-----------------------------------------------------------------------------
float CRobot::DistanceToNearestWall( const Vec2D& aPoint,
                                     const std::vector<Vec2D>& aWalls )
{
    float ClosestDistance = 999.0f;

    for( size_t i = 0; i < aWalls.size(); i++ )
    {
        const Vec2D& Start = aWalls[i];
        const Vec2D& End = aWalls[ (i + 1) % aWalls.size() ];

        float SegmentX = End.x - Start.x;
        float SegmentY = End.y - Start.y;
        float ToPointX = aPoint.x - Start.x;
        float ToPointY = aPoint.y - Start.y;

        float SegmentLengthSquared = SegmentX * SegmentX + SegmentY * SegmentY;
        float Fraction = 0.0f;
        if( SegmentLengthSquared > 0.0f )
        {
            Fraction = ( ToPointX * SegmentX + ToPointY * SegmentY ) / SegmentLengthSquared;
        }

        // Clamp so the closest point is never past either end of the segment.
        if( Fraction < 0.0f ) Fraction = 0.0f;
        if( Fraction > 1.0f ) Fraction = 1.0f;

        float ClosestX = Start.x + Fraction * SegmentX;
        float ClosestY = Start.y + Fraction * SegmentY;
        float DiffX = aPoint.x - ClosestX;
        float DiffY = aPoint.y - ClosestY;
        float Distance = std::sqrt( DiffX * DiffX + DiffY * DiffY );

        if( Distance < ClosestDistance )
        {
            ClosestDistance = Distance;
        }
    }

    return ClosestDistance;
}

//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
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
