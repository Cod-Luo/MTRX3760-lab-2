//-----------------------------------------------------------------------------
// CLineRobot.h
//
// Declares the line-following robot used in A2. The robot has two line sensors,
// two independently controlled wheels, and records its complete trajectory.
//-----------------------------------------------------------------------------
#ifndef CLINEROBOT_H
#define CLINEROBOT_H

#include "CLineSensor.h"

//-----------------------------------------------------------------------------
// CLineRobot follows the floor line using only its two binary sensor readings.
// It also detects wall collisions and decides when it has completed one lap.
//-----------------------------------------------------------------------------
class CLineRobot
{
    public:
        // Creates a radius-15 robot at the map's starting pose. The heading is
        // measured clockwise in radians from the positive x direction.
        CLineRobot( const Vec2D& aStartPosition, float aStartHeading );

        // Reads both line sensors, steers the wheels, and advances the robot by
        // one fixed 0.03-second timestep. aWalls is used only for collisions.
        void Update( const std::vector<Vec2D>& aLine, const std::vector<Vec2D>& aWalls );

        // Draws the complete trail, circular body, and heading indicator.
        void Draw( CRender& aRender ) const;

        // Reports whether the robot has left and returned to its starting area.
        bool HasCompletedLap() const;

        // Prints the final update, collision, and lap-completion totals.
        void PrintSummary() const;

    private:
        void Steer( const std::vector<Vec2D>& aLine );
        void CheckLap();
        void CheckCollision( const std::vector<Vec2D>& aWalls );

        //---Pose and sensing---
        Vec2D mPosition;
        float mHeading;
        Vec2D mStartPosition;
        CLineSensor mCentreSensor;
        CLineSensor mSideSensor;

        //---Motion and trail---
        float mLeftSpeed;
        float mRightSpeed;
        float mDistanceTravelled;
        std::vector<Vec2D> mTrail;

        //---Run monitoring---
        int mUpdates;
        int mCollisions;
        bool mWasColliding;
        bool mLeftStart;
        bool mCompleted;

        static const float Radius;
};

#endif
