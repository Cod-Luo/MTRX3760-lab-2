// CNoisyMotion.h - Shared differential drive, noisy pose, trail and lap state.
#ifndef CNOISYMOTION_H
#define CNOISYMOTION_H

#include "CRender.h"
#include "CNoise.h"
#include <vector>

class CNoisyMotion
{
    public:
        // The supplied courses start near vertex zero. aReverseCourse selects
        // the wall follower's reverse vertex order for lap monitoring only.
        // Travel/return thresholds and checkpoint radius use map distance units.
        CNoisyMotion( const Vec2D& aStart, float aHeading, unsigned int aSeed,
                      float aMinimumTravel, float aReturnDistance,
                      bool aReverseCourse, float aCheckpointRadius );
        // Wheel speeds are converted to travel using a fixed 0.03-second step.
        // aCourse is used only to check progress after movement, never to steer.
        void Advance( float aLeftSpeed, float aRightSpeed, const std::vector<Vec2D>& aCourse );
        const Vec2D& GetPosition() const;
        float GetHeading() const;
        bool HasCompletedLap() const;
        int GetUpdateCount() const;
        void DrawTrail( CRender& aRender, CRender::Colour aColour ) const;
        void DrawBody( CRender& aRender, CRender::Colour aBodyColour,
                       CRender::Colour aHeadingColour ) const;

    private:
        void CheckLap( const std::vector<Vec2D>& aCourse );
        CNoise mNoise;
        Vec2D mPosition;
        float mHeading;
        Vec2D mStart;
        float mMinimumTravel;
        float mReturnDistance;
        bool mReverseCourse;            // traversal order for completion monitoring
        float mCheckpointRadius;        // allows clearance and noisy cornering
        std::size_t mCheckpointsVisited; // prevents shortcuts from counting as laps
        float mTravel;
        bool mLeftStart;
        bool mCompleted;
        int mUpdates;
        std::vector<Vec2D> mTrail;
};
#endif
