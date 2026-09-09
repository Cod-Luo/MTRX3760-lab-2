#include "CSensor.h"
#include <cmath>

CSensor::CSensor( float aAngleOffsetDeg )
    : mAngleOffsetDeg( aAngleOffsetDeg )
{
}

float CSensor::GetDistance( const Vec2D& aRobotPos, float aRobotHeading,
                             const std::vector<Vec2D>& aWalls ) const
{
    // Combine the robot's heading with this sensor's own fixed offset to
    // get the sensor's true direction in world space, then convert that
    // angle into a unit direction vector.
    
    const float DegToRad = 3.14159265f / 180.0f;
    float TotalAngle = aRobotHeading + mAngleOffsetDeg * DegToRad;

    Vec2D RayDirection;
    RayDirection.x = std::cos( TotalAngle );
    RayDirection.y = std::sin( TotalAngle );

    float ClosestDistance = -1.0f;
    
    
    // Check the ray against every wall segment (each pair of consecutive
    // vertices, wrapping back to the first), keeping the closest hit.
    for( size_t i = 0; i < aWalls.size(); i++ )
    {
        Vec2D WallStart = aWalls[i];
        Vec2D WallEnd = aWalls[ (i + 1) % aWalls.size() ];

        Vec2D v2;
        v2.x = WallEnd.x - WallStart.x;
        v2.y = WallEnd.y - WallStart.y;

        Vec2D v1;
        v1.x = aRobotPos.x - WallStart.x;
        v1.y = aRobotPos.y - WallStart.y;

        Vec2D v3;
        v3.x = -RayDirection.y;
        v3.y = RayDirection.x;

        float Denom = v2.x * v3.x + v2.y * v3.y;

        if( Denom != 0.0f )
        {
            float t = ( v2.x * v1.y - v2.y * v1.x ) / Denom;
            float u = ( v1.x * v3.x + v1.y * v3.y ) / Denom;
            
            // t is the distance along the ray to the hit point; u tells us
            // whether that hit point actually falls on this wall segment
            // (between 0 and 1), rather than past one of its ends.
            
            if( t >= 0.0f && u >= 0.0f && u <= 1.0f )
            {
                if( ClosestDistance < 0.0f || t < ClosestDistance )
                {
                    ClosestDistance = t;
                }
            }
        }
    }

    if( ClosestDistance < 0.0f )
    {
        ClosestDistance = 999.0f; // no wall found in range
    }

    return ClosestDistance;
}
