// CNoise.cpp - Uses the rand() approach shown in the lectures and example code.
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
    const float MaximumOffset = 2.0f;
    return Sample( MaximumOffset );
}

float CNoise::HeadingOffset()
{
    const float MaximumRadians = 5.0f * 3.14159265f / 180.0f;
    return Sample( MaximumRadians );
}

void CNoise::PerturbTravel( float& aLeft, float& aRight )
{
    aLeft += Sample( mMaximumTravelOffset );
    aRight += Sample( mMaximumTravelOffset );
}
