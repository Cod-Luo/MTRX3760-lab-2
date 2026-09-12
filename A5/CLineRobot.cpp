// CLineRobot.cpp - The unchanged A2 sensor logic drives independently noisy wheels.
#include "CLineRobot.h"

CLineRobot::CLineRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed )
    // Require 900 units of travel and a return within 8 units of the start.
    : mMotion( aStart, aHeading, aSeed, 900.0f, 8.0f ),
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
        if( !Centre )
        {
            Correction = Side ? Turn : -Turn;
        }
        mMotion.Advance( Speed + Correction, Speed - Correction );
    }
}

bool CLineRobot::HasCompletedLap() const
{
    return mMotion.HasCompletedLap();
}

void CLineRobot::Draw( CRender& aRender, CRender::Colour aTrailColour,
                       CRender::Colour aBodyColour,
                       CRender::Colour aHeadingColour ) const
{
    mMotion.DrawTrail( aRender, aTrailColour );
    mMotion.DrawBody( aRender, aBodyColour, aHeadingColour );
}
