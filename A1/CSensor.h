#ifndef CSENSOR_H
#define CSENSOR_H

//-----------------------------------------------------------------------------
// CSensor.h
//
// A single range-finder sensor, mounted on a robot at a fixed angle relative
// to whichever way the robot is facing. It measures the straight-line
// distance from the robot to the nearest wall along its ray.
//-----------------------------------------------------------------------------

#include "CRender.h"

#include <vector>

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
        //---This sensor's own fixed mounting angle---
        float mAngleOffsetDeg; // degrees, relative to the robot's heading
};

#endif