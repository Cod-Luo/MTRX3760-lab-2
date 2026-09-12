#ifndef CROBOT_H
#define CROBOT_H

//-----------------------------------------------------------------------------
// CRobot.h
//
// Declares the wall-following robot used in A1. The robot carries two range
// sensors aimed to its right, steers using their readings, and records its
// complete trajectory, collisions, and lap completion.
//-----------------------------------------------------------------------------

#include "CRender.h"
#include "CSensor.h"

#include <vector>

//-----------------------------------------------------------------------------
// CRobot follows the wall on its right hand side using two range sensors.
// It also detects collisions against the whole robot body (not just its
// sensor rays), and decides when it has completed one lap.
//-----------------------------------------------------------------------------
class CRobot
{
    public:
        //---Ctor---
        // Creates a radius-15 robot at aStartPos, facing aStartHeading
        // (in radians, clockwise from the positive x axis).
        CRobot( const Vec2D& aStartPos, float aStartHeading );

        //---Simulation---
        // Advances the simulation by one fixed timestep: reads the sensors,
        // decides new wheel speeds, moves the robot, and checks for
        // collisions and lap completion.
        void Update( const std::vector<Vec2D>& aWalls );

        //---Drawing---
        // Draws the robot's trail, body, and heading indicator.
        void Draw( CRender& aRender ) const;

        //---Sensing---
        // Returns this robot's current 90-degree and 45-degree sensor
        // readings, against the given walls.
        float GetSensor90Distance( const std::vector<Vec2D>& aWalls ) const;
        float GetSensor45Distance( const std::vector<Vec2D>& aWalls ) const;

        //---Run status---
        // Returns how many collisions, and how many updates, have happened.
        int GetCollisionCount() const;
        int GetUpdateCount() const;

        // Returns true once the robot has left its start position and
        // returned to it again, completing one full lap.
        bool HasCompletedLap() const;

    private:
        //---Internal steps of Update, split out for readability---
        void Steer( const std::vector<Vec2D>& aWalls );
        void CheckCollision( const std::vector<Vec2D>& aWalls );
        void CheckLap();

        //---Collision geometry---
        // Distance from aPoint to the nearest point on the wall loop aWalls,
        // checking every segment (not just where a sensor happens to point).
        static float DistanceToNearestWall( const Vec2D& aPoint,
                                            const std::vector<Vec2D>& aWalls );

        //---Pose and sensing---
        Vec2D mPosition; // the robot's current x/y position
        float mHeading;  // the robot's current facing direction, in radians

        CSensor mSensor90; // sensor pointing 90 degrees off the robot's heading
        CSensor mSensor45; // sensor pointing 45 degrees off the robot's heading

        //---Motion---
        float mLeftWheelSpeed;  // current speed of the left wheel
        float mRightWheelSpeed; // current speed of the right wheel

        //---Trail and run monitoring---
        std::vector<Vec2D> mTrail; // every position the robot has visited
        int mCollisionCount;       // number of separate collisions so far
        int mUpdateCount;          // number of Update() calls so far
        bool mWasColliding;        // whether the robot was colliding last update

        Vec2D mStartPosition; // where the robot started, used to detect a lap
        bool mHasLeftStart;   // whether the robot has moved away from the start
        bool mLapCompleted;   // whether a full lap has been completed
        float mDistanceTravelled;  // total path length covered so far

        //---Encapsulated constant---
        static const float Radius; // the robot's body radius, in units
};

#endif
