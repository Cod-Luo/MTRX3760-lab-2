//-----------------------------------------------------------------------------
// CNoisyMotion.cpp
//
// Implements the common noisy differential-drive model so both controllers
// experience the same physical assumptions without duplicating motion code.
//-----------------------------------------------------------------------------
#include "CNoisyMotion.h"
#include <cmath>

const float CNoisyMotion::WheelNoiseScale = 1.2f;

CNoisyMotion::CNoisyMotion( const Vec2D& aStart, float aHeading,
                          float aMinimumTravel,
                          float aReturnDistance, float aWheelNoise )
    : mNoise( WheelNoiseScale * aWheelNoise ), mPosition( aStart ),
      mHeading( aHeading ), mStart( aStart ),
      mMinimumTravel( aMinimumTravel ), mReturnDistance( aReturnDistance ),
      mTravel( 0.0f ), mLeftStart( false ), mCompleted( false ), mTrail()
{
    // Each robot receives its own independent x, y and heading samples before
    // sensing begins. The perturbed pose becomes its lap-completion reference.
    mPosition.x += mNoise.PositionOffset();
    mPosition.y += mNoise.PositionOffset();
    mHeading += mNoise.HeadingOffset();
    mStart = mPosition;
    mTrail.push_back( mPosition );
}

void CNoisyMotion::Advance( float aLeftSpeed, float aRightSpeed )
{
    if( !mCompleted )
    {
        const float TimeStep = 0.03f;
        // Twice the required 15-unit radius places one wheel on either side.
        const float WheelBase = 30.0f;
        float LeftTravel = aLeftSpeed * TimeStep;
        float RightTravel = aRightSpeed * TimeStep;
        mNoise.PerturbTravel( LeftTravel, RightTravel );

        const float Travel = (LeftTravel + RightTravel) / 2.0f;
        // The mean wheel travel advances the centre; their difference rotates
        // the robot according to the differential-drive model.
        mHeading += (LeftTravel - RightTravel) / WheelBase;
        mPosition.x += Travel * std::cos( mHeading );
        mPosition.y += Travel * std::sin( mHeading );
        mTravel += std::fabs( Travel );
        CheckLap();
        mTrail.push_back( mPosition );
    }
}

void CNoisyMotion::CheckLap()
{
    const float LeaveDistance = 100.0f;
    const float Dx = mPosition.x - mStart.x;
    const float Dy = mPosition.y - mStart.y;
    const float DistanceSquared = Dx * Dx + Dy * Dy;
    if( DistanceSquared > LeaveDistance * LeaveDistance )
    {
        mLeftStart = true;
    }
    // Leaving the start and travelling the course length prevents small local
    // movements from being mistaken for a completed lap.
    if( mLeftStart && mTravel > mMinimumTravel
        && DistanceSquared < mReturnDistance * mReturnDistance )
    {
        mCompleted = true;
    }
}

const Vec2D& CNoisyMotion::GetPosition() const
{
    return mPosition;
}
float CNoisyMotion::GetHeading() const
{
    return mHeading;
}
bool CNoisyMotion::HasCompletedLap() const
{
    return mCompleted;
}

void CNoisyMotion::DrawTrail( CRender& aRender, CRender::Colour aColour ) const
{
    aRender.DrawLineStrip( mTrail, aColour );
}

void CNoisyMotion::DrawBody( CRender& aRender, CRender::Colour aBodyColour,
                       CRender::Colour aHeadingColour ) const
{
    const float Radius = 15.0f;
    aRender.DrawCircle( mPosition, int( Radius ), aBodyColour );
    const Vec2D End = { mPosition.x + Radius * std::cos( mHeading ),
                        mPosition.y + Radius * std::sin( mHeading ) };
    aRender.DrawLine( mPosition, End, 2.0f, aHeadingColour );
}
