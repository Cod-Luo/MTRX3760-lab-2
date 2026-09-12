//-----------------------------------------------------------------------------
// CSensor.h
//
// Declares one range sensor mounted at a fixed angle on the wall-following
// robot. It reports the first wall intersection along its sensing ray.
//-----------------------------------------------------------------------------

#ifndef CSENSOR_H
#define CSENSOR_H

#include "CRender.h"
#include <vector>

//-----------------------------------------------------------------------------
// CSensor converts its mounting angle and the robot heading into a world-space
// ray, then measures that ray against every segment of the wall loop.
//-----------------------------------------------------------------------------
class CSensor
{
    public:
        //---Ctor---
        // Creates a sensor mounted at aAngleOffsetDeg degrees relative to
        // whatever the robot it belongs to is facing.
        CSensor( float aAngleOffsetDeg );

        //---Sensing---
        // Returns the distance from aRobotPos to the nearest wall in
        // aWalls, along this sensor's ray. aRobotHeading is the robot's
        // current facing direction, in radians.
        float GetDistance( const Vec2D& aRobotPos, float aRobotHeading,
                            const std::vector<Vec2D>& aWalls ) const;

    private:
        // Fixed clockwise mounting angle relative to the robot's heading.
        float mAngleOffsetDeg;
};

#endif
