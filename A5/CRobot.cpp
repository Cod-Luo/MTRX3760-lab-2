// CRobot.cpp - Sensor feedback steers; wheel noise is applied after control.
#include "CRobot.h"

CRobot::CRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed )
    // Require 1500 units of travel and a return within 15 units of the start.
    : mMotion( aStart, aHeading, aSeed, 1500.0f, 15.0f ),
      mSensor90( 90.0f ), mSensor45( 45.0f )
{
}

void CRobot::Update( const std::vector<Vec2D>& aWalls )
{
    if( !mMotion.HasCompletedLap() )
    {
        const float BaseSpeed = 42.0f;
        const float TargetSideDistance = 45.0f;
        const float TargetDiagonalDistance = TargetSideDistance * 1.41421356f;
        const float MaxSensorDistance = 120.0f;
        // Gentler feedback leaves the wheel disturbances visible while still
        // correcting the course using the same two sensor errors as A2.
        const float SideGain = 0.45f;
        const float DiagonalGain = 0.9f;
        const float MaxTurn = 157.5f;
        float Dist90 = mSensor90.GetDistance( mMotion.GetPosition(), mMotion.GetHeading(), aWalls );
        float Dist45 = mSensor45.GetDistance( mMotion.GetPosition(), mMotion.GetHeading(), aWalls );
        if( Dist90 > MaxSensorDistance )
        {
            Dist90 = MaxSensorDistance;
        }
        if( Dist45 > MaxSensorDistance )
        {
            Dist45 = MaxSensorDistance;
        }
        float Turn = SideGain * (Dist90 - TargetSideDistance)
                   + DiagonalGain * (Dist45 - TargetDiagonalDistance);
        if( Turn > MaxTurn )
        {
            Turn = MaxTurn;
        }
        if( Turn < -MaxTurn )
        {
            Turn = -MaxTurn;
        }
        mMotion.Advance( BaseSpeed + Turn, BaseSpeed - Turn );
    }
}

bool CRobot::HasCompletedLap() const
{
    return mMotion.HasCompletedLap();
}

void CRobot::Draw( CRender& aRender, CRender::Colour aTrailColour,
                   CRender::Colour aBodyColour,
                   CRender::Colour aHeadingColour ) const
{
    mMotion.DrawTrail( aRender, aTrailColour );
    mMotion.DrawBody( aRender, aBodyColour, aHeadingColour );
}
