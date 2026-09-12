// CLineRobot.h - A2's two binary line sensors with shared noisy motion.
#ifndef CLINEROBOT_H
#define CLINEROBOT_H
#include "CLineSensor.h"
#include "CNoisyMotion.h"

class CLineRobot
{
    public:
        // Creates a line follower at the supplied map pose. Heading is radians.
        CLineRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed );

        // Reads both line sensors, chooses wheel speeds, and advances one step.
        void Update( const std::vector<Vec2D>& aLine );

        // Reports progress without exposing the robot's internal motion object.
        bool HasCompletedLap() const;

        // Draws the complete trail, body and contrasting heading indicator.
        void Draw( CRender& aRender, CRender::Colour aTrailColour,
                   CRender::Colour aBodyColour,
                   CRender::Colour aHeadingColour ) const;
    private:
        CNoisyMotion mMotion;
        CLineSensor mCentreSensor;
        CLineSensor mSideSensor;
};
#endif
