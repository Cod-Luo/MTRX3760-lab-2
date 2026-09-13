// CNoise.h - Random starting-pose and wheel-travel offsets for A5.
#ifndef CNOISE_H
#define CNOISE_H

class CNoise
{
    public:
        CNoise( float aMaximumTravelOffset );

        // Position offsets are in map units; heading offsets are in radians.
        float PositionOffset();
        float HeadingOffset();

        // Adds an independent configured offset to each wheel's travel.
        void PerturbTravel( float& aLeft, float& aRight );

    private:
        // Returns a random value between -aMagnitude and +aMagnitude.
        float Sample( float aMagnitude );
        float mMaximumTravelOffset;
};
#endif
