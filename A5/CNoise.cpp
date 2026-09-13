//-----------------------------------------------------------------------------
// CNoise.cpp
//
// Implements bounded random pose offsets and slowly varying wheel slip used
// to produce a believable spread of A5 trajectories.
//-----------------------------------------------------------------------------
#include "CNoise.h"
#include <cstdlib>

CNoise::CNoise( float aMaximumTravelOffset )
    : mMaximumTravelOffset( aMaximumTravelOffset ),
      mLeftTravelOffset( Sample( aMaximumTravelOffset ) ),
      mRightTravelOffset( Sample( aMaximumTravelOffset ) )
{
}

float CNoise::Sample( float aMagnitude )
{
    const float Fraction = float( std::rand() ) / float( RAND_MAX );
    return aMagnitude * (2.0f * Fraction - 1.0f);
}

float CNoise::PositionOffset()
{
    // Two map units vary the starting point without moving a robot away from
    // the wall or line that its controller must initially detect.
    const float MaximumOffset = 2.0f;
    return Sample( MaximumOffset );
}

float CNoise::HeadingOffset()
{
    // A five-degree error is visible over a lap but small enough for both
    // sensor-driven controllers to recover reliably.
    const float MaximumRadians = 5.0f * 3.14159265f / 180.0f;
    return Sample( MaximumRadians );
}

void CNoise::PerturbTravel( float& aLeft, float& aRight )
{
    // Slowly varying slip persists long enough to alter the visible path. A
    // fresh independent sample still changes each wheel on every update.
    const float Retention = 0.98f;
    const float ChangeMagnitude = 0.10f * mMaximumTravelOffset;
    mLeftTravelOffset = Retention * mLeftTravelOffset + Sample( ChangeMagnitude );
    mRightTravelOffset = Retention * mRightTravelOffset + Sample( ChangeMagnitude );

    if( mLeftTravelOffset > mMaximumTravelOffset )
    {
        mLeftTravelOffset = mMaximumTravelOffset;
    }
    if( mLeftTravelOffset < -mMaximumTravelOffset )
    {
        mLeftTravelOffset = -mMaximumTravelOffset;
    }
    if( mRightTravelOffset > mMaximumTravelOffset )
    {
        mRightTravelOffset = mMaximumTravelOffset;
    }
    if( mRightTravelOffset < -mMaximumTravelOffset )
    {
        mRightTravelOffset = -mMaximumTravelOffset;
    }

    aLeft += mLeftTravelOffset;
    aRight += mRightTravelOffset;
}
