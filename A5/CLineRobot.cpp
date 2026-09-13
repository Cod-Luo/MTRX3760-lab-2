//-----------------------------------------------------------------------------
// CLineRobot.cpp
//
// Implements the two-sensor line controller used by every noisy A5 line robot.
// Steering depends only on current sensor readings and remembered recovery
// direction; the controller never reads map vertices to choose its path.
//-----------------------------------------------------------------------------
#include "CLineRobot.h"

CLineRobot::CLineRobot( const Vec2D& aStart, float aHeading )
    // Forward-mounted sensors detect corners shortly before the centre arrives.
    // A 15-unit return region tolerates noise while 500 units prevents false laps.
    : mMotion( aStart, aHeading, 500.0f, 15.0f, 1.0f ),
      mCentreSensor( 4.0f, 0.0f ), mSideSensor( 4.0f, 5.0f ), mTurnRight( false )
{
}

void CLineRobot::Update( const std::vector<Vec2D>& aLine )
{
    if( !mMotion.HasCompletedLap() )
    {
        const float Speed = 30.0f;
        // Corrections are strong enough to negotiate each straight-segment
        // corner; slightly lower forward speed reduces corner overshoot.
        const float Turn = 45.0f;
        const float CornerSpeed = 27.0f;
        const bool Centre = mCentreSensor.Read( mMotion.GetPosition(), mMotion.GetHeading(), aLine );
        const bool Side = mSideSensor.Read( mMotion.GetPosition(), mMotion.GetHeading(), aLine );
        float Correction = 0.0f;
        // The side sensor can detect a right corner while the centre sensor
        // still sees the incoming segment, so give that reading priority.
        if( Side )
        {
            mTurnRight = true;
            Correction = Turn;
        }
        else if( !Centre )
        {
            // Keep the last recovery direction across a gap at the corner.
            Correction = mTurnRight ? Turn : -Turn;
        }
        else
        {
            // Centre-only contact restores normal tracking.
            mTurnRight = false;
        }
        // Slower cornering prevents the robot overshooting the outgoing line.
        const float ForwardSpeed = Correction == 0.0f ? Speed : CornerSpeed;
        mMotion.Advance( ForwardSpeed + Correction, ForwardSpeed - Correction );
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
