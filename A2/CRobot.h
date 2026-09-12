#ifndef CROBOT_H
#define CROBOT_H

#include "CRender.h"
#include "CSensor.h"
#include "CLoopReader.h"

#include <vector>
//---CRobot Interface------------------------------------------------------
// CRobot simulates a two-wheeled robot that follows a wall on its right
// hand side, using two range sensors (at 90 and 45 degrees) to steer.


class CRobot
{
    public:
        // Creates a robot starting at aStartPos, facing aStartHeading
        // (in radians).
        CRobot( const Vec2D& aStartPos, float aStartHeading );

        
        // Advances the simulation by one fixed timestep: reads the
        // sensors, decides new wheel speeds, and moves the robot.
        void Update( const std::vector<Vec2D>& aWalls );
        
        
        // Draws the robot's trail, body, and heading indicator.
        void Draw( CRender& aRender ) const;
        
        // Returns this robot's current 90-degree and 45-degree sensor
        // readings, against the given walls.
        float GetSensor90Distance( const std::vector<Vec2D>& aWalls ) const;
	float GetSensor45Distance( const std::vector<Vec2D>& aWalls ) const;
	
	// Returns how many collisions, and how many updates, have happened so far.
	int GetCollisionCount() const;
        int GetUpdateCount() const;
        
        
        // Returns true once the robot has left its start position and
        // returned to it again, completing one full lap.
        bool HasCompletedLap() const;

    private:
        Vec2D mPosition; // the robot's current x/y position
        float mHeading;  // the robot's current facing direction, in radians

        CSensor mSensor90; // sensor pointing 90 degrees off the robot's heading
        CSensor mSensor45; // sensor pointing 45 degrees off the robot's heading

        float mLeftWheelSpeed;  // current speed of the left wheel
        float mRightWheelSpeed; // current speed of the right wheel

        std::vector<Vec2D> mTrail; // every position the robot has visited
        int mCollisionCount;       // number of separate collisions so far
        int mUpdateCount;          // number of Update() calls so far
        bool mWasColliding;        // whether the robot was colliding last update

        Vec2D mStartPosition; // where the robot started, used to detect a lap
        bool mHasLeftStart;   // whether the robot has moved away from the start
        bool mLapCompleted;   // whether a full lap has been completed
        float mDistanceTravelled; // total path length covered so far
};

#endif
