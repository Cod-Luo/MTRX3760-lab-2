// CNoise.cpp - Wheel calibration error plus fresh zero-mean slip each update.
#include "CNoise.h"

CNoise::CNoise( unsigned int aSeed )
    : mGenerator( aSeed ), mUniform( -1.0f, 1.0f ),
      mLeftBias( Sample( 0.04f ) ), mRightBias( Sample( 0.04f ) )
{
}

float CNoise::Sample( float aMagnitude )
{
    return aMagnitude * mUniform( mGenerator );
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
    const float StepVariation = 0.15f;
    aLeft *= 1.0f + mLeftBias + Sample( StepVariation );
    aRight *= 1.0f + mRightBias + Sample( StepVariation );
}
