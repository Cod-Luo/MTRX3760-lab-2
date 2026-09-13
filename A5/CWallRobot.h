//-----------------------------------------------------------------------------
// CWallRobot.h
//
// Declares the A5 wall follower. Its two right-facing range sensors control
// independently driven wheels through the shared noisy-motion model.
//-----------------------------------------------------------------------------
#ifndef CROBOT_H
#define CROBOT_H
#include "CSensor.h"
#include "CNoisyMotion.h"

class CWallRobot
{
    public:
        // Creates a wall follower at the supplied map pose. Heading is radians.
        CWallRobot( const Vec2D& aStart, float aHeading );

        // Reads both range sensors, chooses wheel speeds, and advances one step.
        void Update( const std::vector<Vec2D>& aWalls );

        // Reports progress without exposing the robot's internal motion object.
        bool HasCompletedLap() const;

        // Draws the complete trail, body and contrasting heading indicator.
        void Draw( CRender& aRender, CRender::Colour aTrailColour,
                   CRender::Colour aBodyColour,
                   CRender::Colour aHeadingColour ) const;
    private:
        // Motion owns the perturbed pose, wheel model, trail and lap state.
        CNoisyMotion mMotion;
        // The required sensors face directly right and forward-right.
        CSensor mSensor90;
        CSensor mSensor45;
};
#endif
