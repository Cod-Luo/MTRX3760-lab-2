#include "CRobot.h"
#include <cmath>
#include <iostream>


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
{
}


void CRobot::Update( const std::vector<Vec2D>& aWalls )
{
    const float dt = 0.03f;
    const float WheelBase = 30.0f;
    const float BaseSpeed = 0.8f;

    const float TargetSideDistance = 40.0f;
    const float TargetDiagonalDistance = TargetSideDistance * 1.41421356f;
    const float MaxSensorDistance = 120.0f;

    const float SideGain = 0.04f;
    const float DiagonalGain = 0.08f;
    const float MaxTurn = 3.0f;

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

    mTrail.push_back( mPosition );
    mUpdateCount++;


    // A collision is when either sensor reports a wall closer than the
    // robot's own radius. Only count a new collision when it *starts*
    // touching, not every update it remains touching.
    const float RobotRadius = 15.0f;
    bool IsColliding = ( Dist90 < RobotRadius || Dist45 < RobotRadius );
    if( IsColliding && !mWasColliding )
    {
        mCollisionCount++;
        std::cout << "Collision! Total so far: " << mCollisionCount << std::endl;
    }
    mWasColliding = IsColliding;

    // A lap is detected by leaving the start position by some distance,
    // then returning close to it again.
    const float LeaveThreshold = 100.0f;
    const float ReturnThreshold = 30.0f;

    float dx = mPosition.x - mStartPosition.x;
    float dy = mPosition.y - mStartPosition.y;
    float DistanceFromStart = std::sqrt( dx * dx + dy * dy );

    if( !mHasLeftStart && DistanceFromStart > LeaveThreshold )
    {
        mHasLeftStart = true;
    }

    if( mHasLeftStart && !mLapCompleted && DistanceFromStart < ReturnThreshold )
    {
        mLapCompleted = true;
        std::cout << "Lap completed!" << std::endl;
    }
}

void CRobot::Draw( CRender& aRender ) const
{
    for( size_t i = 1; i < mTrail.size(); i++ )
    {
        aRender.DrawLine( mTrail[i - 1], mTrail[i], 1.0f, YELLOW );
    }

    const float Radius = 15.0f;
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






