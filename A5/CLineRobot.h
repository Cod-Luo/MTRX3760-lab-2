//-----------------------------------------------------------------------------
// CLineRobot.h
//
// Declares the A5 line follower. Two binary floor sensors select independently
// driven wheel speeds while shared noisy motion perturbs its actual travel.
//-----------------------------------------------------------------------------
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
        // Motion owns the perturbed pose, wheel model, trail and lap state.
        CNoisyMotion mMotion;
        // One sensor tracks the line and the offset sensor recognises corners.
        CLineSensor mCentreSensor;
        CLineSensor mSideSensor;
        // Remembers a right-hand recovery while both sensors are off the line.
        bool mTurnRight;
};
#endif
