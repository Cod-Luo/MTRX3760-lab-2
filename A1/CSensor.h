#ifndef CSENSOR_H
#define CSENSOR_H

#include "CRender.h"
#include <vector>

//---CSensor Interface---------------------------------------------------
// CSensor represents a single range-finder sensor mounted on a robot at a
// fixed angle. It measures the distance to the nearest wall along its ray.
class CSensor
{
    public:
        // Creates a sensor mounted at aAngleOffsetDeg degrees relative to
        // whatever the robot it belongs to is facing.
        CSensor( float aAngleOffsetDeg );

        // Returns the distance from aRobotPos to the nearest wall in
        // aWalls, along this sensor's ray. aRobotHeading is the robot's
        // current facing direction, in radians.
        float GetDistance( const Vec2D& aRobotPos, float aRobotHeading,
                            const std::vector<Vec2D>& aWalls ) const;

    private:
        float mAngleOffsetDeg; // this sensor's fixed angle, relative to the robot
};

#endif
