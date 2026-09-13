// CLineRobot.h - Two binary line sensors and corner recovery under noise.
#ifndef CLINEROBOT_H
#define CLINEROBOT_H
#include "CLineSensor.h"
#include "CNoisyMotion.h"

class CLineRobot
{
    public:
        // Creates a line follower at the supplied map pose. Heading is radians.
        CLineRobot( const Vec2D& aStart, float aHeading );

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
        // Remembers a right-hand recovery while both sensors are off the line.
        bool mTurnRight;
};
#endif
