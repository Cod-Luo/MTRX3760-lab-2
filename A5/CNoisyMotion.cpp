// CNoisyMotion.cpp - Both controllers share the same physical noise model.
#include "CNoisyMotion.h"
#include <cmath>

CNoisyMotion::CNoisyMotion( const Vec2D& aStart, float aHeading,
                          unsigned int aSeed, float aMinimumTravel,
                          float aReturnDistance, bool aReverseCourse,
                          float aCheckpointRadius )
    : mNoise( aSeed ), mPosition( aStart ), mHeading( aHeading ), mStart( aStart ),
      mMinimumTravel( aMinimumTravel ), mReturnDistance( aReturnDistance ),
      mReverseCourse( aReverseCourse ), mCheckpointRadius( aCheckpointRadius ),
      mCheckpointsVisited( 0 ),
      mTravel( 0.0f ), mLeftStart( false ), mCompleted( false ), mUpdates( 0 ), mTrail()
{
    mPosition.x += mNoise.PositionOffset();
    mPosition.y += mNoise.PositionOffset();
    mHeading += mNoise.HeadingOffset();
    mStart = mPosition;
    mTrail.push_back( mPosition );
}

void CNoisyMotion::Advance( float aLeftSpeed, float aRightSpeed,
                           const std::vector<Vec2D>& aCourse )
{
    if( !mCompleted )
    {
        const float TimeStep = 0.03f;
        const float WheelBase = 30.0f;
        float LeftTravel = aLeftSpeed * TimeStep;
        float RightTravel = aRightSpeed * TimeStep;
        mNoise.PerturbTravel( LeftTravel, RightTravel );
        const float Travel = (LeftTravel + RightTravel) / 2.0f;
        mHeading += (LeftTravel - RightTravel) / WheelBase;
        mPosition.x += Travel * std::cos( mHeading );
        mPosition.y += Travel * std::sin( mHeading );
        mTravel += std::fabs( Travel );
        ++mUpdates;
        CheckLap( aCourse );
        mTrail.push_back( mPosition );
    }
}

void CNoisyMotion::CheckLap( const std::vector<Vec2D>& aCourse )
{
    // The supplied maps start beside vertex zero. Require each subsequent
    // corner in traversal order, then vertex zero again. This monitoring data
    // never changes a wheel command: only the sensors determine the route.
    if( mCheckpointsVisited < aCourse.size() )
    {
        const std::size_t Next = mCheckpointsVisited + 1;
        const std::size_t Index = mReverseCourse ? (aCourse.size() - Next) % aCourse.size()
                                                : Next % aCourse.size();
        const float CheckDx = mPosition.x - aCourse[Index].x;
        const float CheckDy = mPosition.y - aCourse[Index].y;
        if( CheckDx * CheckDx + CheckDy * CheckDy < mCheckpointRadius * mCheckpointRadius )
        {
            ++mCheckpointsVisited;
        }
    }
    const float LeaveDistance = 100.0f;
    const float Dx = mPosition.x - mStart.x;
    const float Dy = mPosition.y - mStart.y;
    const float DistanceSquared = Dx * Dx + Dy * Dy;
    if( DistanceSquared > LeaveDistance * LeaveDistance )
    {
        mLeftStart = true;
    }
    if( !aCourse.empty() && mCheckpointsVisited == aCourse.size()
        && mLeftStart && mTravel > mMinimumTravel
        && DistanceSquared < mReturnDistance * mReturnDistance )
    {
        mCompleted = true;
    }
}

const Vec2D& CNoisyMotion::GetPosition() const { return mPosition; }
float CNoisyMotion::GetHeading() const { return mHeading; }
bool CNoisyMotion::HasCompletedLap() const { return mCompleted; }
int CNoisyMotion::GetUpdateCount() const { return mUpdates; }

void CNoisyMotion::DrawTrail( CRender& aRender, CRender::Colour aColour ) const
{
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        aRender.DrawLine( mTrail[i - 1], mTrail[i], 0.7f, aColour );
    }
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
