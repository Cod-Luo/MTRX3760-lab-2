// CRobot.h - A2's two-range-sensor controller with shared noisy motion.
#ifndef CROBOT_H
#define CROBOT_H
#include "CSensor.h"
#include "CNoisyMotion.h"

class CRobot
{
    public:
        // Creates a wall follower at the supplied map pose. Heading is radians.
        CRobot( const Vec2D& aStart, float aHeading );

        // Reads both range sensors, chooses wheel speeds, and advances one step.
        void Update( const std::vector<Vec2D>& aWalls );

        // Reports progress without exposing the robot's internal motion object.
        bool HasCompletedLap() const;

        // Draws the complete trail, body and contrasting heading indicator.
        void Draw( CRender& aRender, CRender::Colour aTrailColour,
                   CRender::Colour aBodyColour,
                   CRender::Colour aHeadingColour ) const;
    private:
        CNoisyMotion mMotion;
        CSensor mSensor90;
        CSensor mSensor45;
};
#endif
