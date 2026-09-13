// CNoisyMotion.h - Shared differential drive, noisy pose, trail and lap state.
#ifndef CNOISYMOTION_H
#define CNOISYMOTION_H

#include "CRender.h"
#include "CNoise.h"
#include <vector>

class CNoisyMotion
{
    public:
        // Travel and return thresholds use map distance units;
        // aHeading is measured in radians clockwise from the positive x axis.
        CNoisyMotion( const Vec2D& aStart, float aHeading,
                      float aMinimumTravel, float aReturnDistance,
                      float aWheelNoise );
        // Wheel speeds are converted to travel using a fixed 0.03-second step.
        void Advance( float aLeftSpeed, float aRightSpeed );

        // Read-only state used by each controller and by the simulation summary.
        const Vec2D& GetPosition() const;
        float GetHeading() const;
        bool HasCompletedLap() const;
        void DrawTrail( CRender& aRender, CRender::Colour aColour ) const;
        void DrawBody( CRender& aRender, CRender::Colour aBodyColour,
                       CRender::Colour aHeadingColour ) const;

    private:
        void CheckLap();
        CNoise mNoise;
        Vec2D mPosition;
        float mHeading;
        Vec2D mStart;
        float mMinimumTravel;
        float mReturnDistance;
        float mTravel;
        bool mLeftStart;
        bool mCompleted;
        std::vector<Vec2D> mTrail;
};
#endif
