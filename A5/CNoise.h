// CNoise.h - Reproducible, independent starting-pose and wheel perturbations.
// Each robot owns its own generator. Position offsets are uniform +/-2 units,
// heading offsets +/-5 degrees, wheel calibration biases +/-4%, and fresh
// per-step wheel variations +/-15%. Wheel error is relative to commanded travel,
// so a stopped wheel stays stopped and the combined error is bounded by 19%.
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
        float mLeftBias;
        float mRightBias;
};
#endif
