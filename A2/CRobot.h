//-----------------------------------------------------------------------------
// CRobot.h
//
// Declares the wall-following robot used alongside the line follower in A2.
// It owns two right-facing range sensors, its motion state, complete trail,
// collision count, and lap-completion state.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CRender.h"
#include "CSensor.h"
#include <vector>

//-----------------------------------------------------------------------------
// CRobot follows the wall on its right using distances reported by sensors at
// 90 and 45 degrees. Its two wheel speeds determine translation and rotation.
//-----------------------------------------------------------------------------
class CRobot
{
    public:
        //---Ctor---
        // Creates a radius-15 robot at aStartPos. aStartHeading is clockwise
        // in radians from the positive x direction.
        CRobot( const Vec2D& aStartPos, float aStartHeading );

        //---Simulation---
        // Reads both range sensors, sets the wheel speeds, advances one fixed
        // timestep, and updates the trail, collisions, and lap state.
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
        // These values are used by CSimulation for the final run summary.
        int GetCollisionCount() const;
        int GetUpdateCount() const;

        // Returns true after the robot has left the starting area, travelled
        // far enough to cover the room circuit, and returned near the start.
        bool HasCompletedLap() const;

    private:
        // Finds the shortest distance from the robot centre to the wall loop.
        // Collision detection uses the whole disc, not only the sensor rays.
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
        float mDistanceTravelled; // total path length, preventing an early lap

        static const float Radius;
};

#endif
