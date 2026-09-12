//-----------------------------------------------------------------------------
// CLineSensor.h
//
// Declares a point sensor mounted at a fixed forward and right-hand offset from
// a robot. The sensor distinguishes the five-unit-wide floor line from floor.
//-----------------------------------------------------------------------------
#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CRender.h"
#include <vector>

//-----------------------------------------------------------------------------
// CLineSensor transforms its mounting point into map coordinates and reports
// whether the floor directly beneath that point is part of the line.
//-----------------------------------------------------------------------------
class CLineSensor
{
    public:
        // Mounts the sensor using distances forward and right from the robot's
        // centre. Negative offsets would place it in the opposite direction.
        CLineSensor( float aForward, float aRight );

        // Returns whether the point beneath this sensor is on the 5-unit-wide line.
        bool Read( const Vec2D& aPosition, float aHeading,
                   const std::vector<Vec2D>& aLine ) const;

        // Shared segment-distance calculation for floor sensing and disc collisions.
        static float DistanceTo( const Vec2D& aPoint, const std::vector<Vec2D>& aLoop );

    private:
        // Sensor position in the robot's local forward/right coordinate frame.
        float mForward;
        float mRight;
};

#endif
