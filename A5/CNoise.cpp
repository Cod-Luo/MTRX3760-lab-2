//-----------------------------------------------------------------------------
// CNoise.cpp
//
// Implements the small, uniformly distributed pose and wheel-travel offsets
// used to produce a believable spread of A5 trajectories.
//-----------------------------------------------------------------------------
#include "CNoise.h"
#include <cstdlib>

CNoise::CNoise( float aMaximumTravelOffset )
    : mMaximumTravelOffset( aMaximumTravelOffset )
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

float CNoise::WheelScale()
{
    // A six-percent calibration range creates persistent lateral variation
    // while leaving the sensor feedback strong enough to complete each course.
    const float MaximumScaleError = 0.50f;
    return 1.0f + Sample( MaximumScaleError );
}

void CNoise::PerturbTravel( float& aLeft, float& aRight )
{
    // Independent samples model the two wheels slipping by different amounts;
    // their difference also introduces a small heading error.
    aLeft += Sample( mMaximumTravelOffset );
    aRight += Sample( mMaximumTravelOffset );
}
