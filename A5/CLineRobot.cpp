// CLineRobot.cpp - The unchanged A2 sensor logic drives independently noisy wheels.
#include "CLineRobot.h"

CLineRobot::CLineRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed )
    // Require 900 units of travel, a return within 8 units, and passage within
    // 25 units of every line vertex in order. Only sensors steer the robot.
    : mMotion( aStart, aHeading, aSeed, 900.0f, 8.0f, false, 25.0f ),
      mCentreSensor( 0.25f, 0.0f ), mSideSensor( 0.25f, 6.0f )
{
}

void CLineRobot::Update( const std::vector<Vec2D>& aLine )
{
    if( !mMotion.HasCompletedLap() )
    {
        const float Speed = 30.0f;
        const float Turn = 70.0f;
        const bool Centre = mCentreSensor.Read( mMotion.GetPosition(), mMotion.GetHeading(), aLine );
        const bool Side = mSideSensor.Read( mMotion.GetPosition(), mMotion.GetHeading(), aLine );
        float Correction = 0.0f;
        if( !Centre ) { Correction = Side ? Turn : -Turn; }
        mMotion.Advance( Speed + Correction, Speed - Correction, aLine );
    }
}

const CNoisyMotion& CLineRobot::GetMotion() const { return mMotion; }
