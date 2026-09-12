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
        //---Internal simulation steps---
        // Converts the two binary readings into left and right wheel speeds.
        void Steer( const std::vector<Vec2D>& aLine );

        // Updates the completion state after the robot moves.
        void CheckLap();

        // Counts the beginning of each contact between the disc and a wall.
        void CheckCollision( const std::vector<Vec2D>& aWalls );

        //---Pose and sensing---
        Vec2D mPosition;      // centre of the robot in map coordinates
        float mHeading;       // clockwise radians from the positive x direction
        Vec2D mStartPosition; // reference point used to recognise a lap
        CLineSensor mCentreSensor; // sensor positioned over the target line
        CLineSensor mSideSensor;   // offset sensor used to choose turn direction

        //---Motion and trail---
        float mLeftSpeed;            // independently controlled left wheel
        float mRightSpeed;           // independently controlled right wheel
        float mDistanceTravelled;    // prevents an early false lap
        std::vector<Vec2D> mTrail;   // retained positions for the complete run

        //---Run monitoring---
        int mUpdates;          // number of fixed simulation steps completed
        int mCollisions;       // number of separate wall contacts
        bool mWasColliding;    // distinguishes a new contact from continued contact
        bool mLeftStart;       // prevents completion at the initial position
        bool mCompleted;       // set once a full lap is recognised

        static const float Radius; // shared 15-unit body radius
};

#endif
