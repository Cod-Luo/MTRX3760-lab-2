//-----------------------------------------------------------------------------
// CNoise.h
//
// Declares the bounded random offsets used to vary each A5 robot's starting
// pose and the distance travelled by its two wheels on every update.
//-----------------------------------------------------------------------------
#ifndef CNOISE_H
#define CNOISE_H

class CNoise
{
    public:
        // Sets the largest positive or negative wheel-travel error, in map
        // units, that PerturbTravel may add during one simulation step.
        CNoise( float aMaximumTravelOffset );

        // Position offsets are in map units; heading offsets are in radians.
        float PositionOffset();
        float HeadingOffset();

        // Adds an independent configured offset to each wheel's travel.
        void PerturbTravel( float& aLeft, float& aRight );

    private:
        // Returns a random value between -aMagnitude and +aMagnitude.
        float Sample( float aMagnitude );

        // Per-step bound shared by the independently sampled wheel errors.
        float mMaximumTravelOffset;
};
#endif
