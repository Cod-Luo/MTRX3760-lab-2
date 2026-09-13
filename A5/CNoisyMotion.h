//-----------------------------------------------------------------------------
// CNoisyMotion.h
//
// Declares the differential-drive motion, noise, trajectory and lap state
// shared by the wall-following and line-following robots in A5.
//-----------------------------------------------------------------------------
#ifndef CNOISYMOTION_H
#define CNOISYMOTION_H

#include "CRender.h"
#include "CNoise.h"
#include <vector>

class CNoisyMotion
{
    public:
        // Creates a robot at a randomly perturbed pose. Travel, return and
        // wheel-noise arguments use map units; heading uses clockwise radians.
        CNoisyMotion( const Vec2D& aStart, float aHeading,
                      float aMinimumTravel, float aReturnDistance,
                      float aWheelNoise );
        // Converts commanded wheel speeds into travel for one fixed 0.03-second
        // update, adds independent wheel errors, and records the new position.
        void Advance( float aLeftSpeed, float aRightSpeed );

        // Read-only state used by each controller and by the simulation summary.
        const Vec2D& GetPosition() const;
        float GetHeading() const;
        bool HasCompletedLap() const;
        void DrawTrail( CRender& aRender, CRender::Colour aColour ) const;
        void DrawBody( CRender& aRender, CRender::Colour aBodyColour,
                       CRender::Colour aHeadingColour ) const;

    private:
        // Scales A5's baseline wheel error to a small travel perturbation.
        static const float WheelNoiseScale;

        // Marks the lap complete only after leaving the start, travelling the
        // configured course distance and returning to the starting region.
        void CheckLap();

        //---Noise, pose and completion configuration---
        CNoise mNoise;
        Vec2D mPosition;
        float mHeading;
        Vec2D mStart;
        float mMinimumTravel;
        float mReturnDistance;

        //---Run state retained for the complete trajectory---
        float mTravel;
        bool mLeftStart;
        bool mCompleted;
        std::vector<Vec2D> mTrail;
};
#endif
