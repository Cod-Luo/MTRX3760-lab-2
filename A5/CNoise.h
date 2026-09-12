// CNoise.h - Reproducible, independent starting-pose and wheel perturbations.
// Each robot owns its own generator. Position offsets are uniform +/-2 units,
// heading offsets +/-5 degrees, and per-step wheel variations are +/-15%.
// Wheel error is relative to commanded travel, so a stopped wheel stays stopped.
#ifndef CNOISE_H
#define CNOISE_H

#include <random>

class CNoise
{
    public:
        explicit CNoise( unsigned int aSeed );

        // Symmetric starting offsets: units for position, radians for heading.
        float PositionOffset();
        float HeadingOffset();

        // Perturbs each wheel's commanded travel independently at every step.
        void PerturbTravel( float& aLeft, float& aRight );

    private:
        float Sample( float aMagnitude );
        std::mt19937 mGenerator;
        std::uniform_real_distribution<float> mUniform;
};
#endif
