//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Implements A2's range-sensor feedback controller on top of A5's noisy
// motion. Noise changes actual wheel travel after the controller issues its
// commands, allowing the controller to respond naturally on the next update.
//-----------------------------------------------------------------------------
#include "CRobot.h"

CRobot::CRobot( const Vec2D& aStart, float aHeading )
    // Preserve A2's minimum travel. The wider return tolerance accommodates
    // the deliberately perturbed starting pose.
    : mMotion( aStart, aHeading, 1500.0f, 15.0f, 1.0f ),
      mSensor90( 90.0f ), mSensor45( 45.0f )
{
}

void CRobot::Update( const std::vector<Vec2D>& aWalls )
{
    if( !mMotion.HasCompletedLap() )
    {
        const float BaseSpeed = 42.0f;
        // The side sensor controls clearance while the diagonal sensor warns
        // of approaching corners. Its target is sqrt(2) times the clearance.
        const float TargetSideDistance = 45.0f;
        const float TargetDiagonalDistance = TargetSideDistance * 1.41421356f;
        const float MaxSensorDistance = 120.0f;
        const float SideGain = 2.1f;
        const float DiagonalGain = 4.2f;
        const float MaxTurn = 157.5f;

        // Limit missing or very distant readings so open space cannot create
        // an unbounded steering correction.
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
        // Positive correction turns right; negative correction turns left.
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
