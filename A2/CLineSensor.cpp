//-----------------------------------------------------------------------------
// CLineSensor.cpp
//
// Implements line sensing by transforming the sensor's mounting offset into
// map coordinates and measuring that point against every floor-line segment.
//-----------------------------------------------------------------------------

#include "CLineSensor.h"
#include <cmath>

//-----------------------------------------------------------------------------
CLineSensor::CLineSensor( float aForward, float aRight )
    : mForward( aForward ), mRight( aRight ) {}

//-----------------------------------------------------------------------------
bool CLineSensor::Read( const Vec2D& aPosition, float aHeading,
                        const std::vector<Vec2D>& aLine ) const
{
    const float HalfLineWidth = 2.5f;

    // Rotate the forward/right mounting offset by the robot heading. Screen y
    // increases downwards, so positive headings and right offsets are clockwise.
    const Vec2D Location = {
        aPosition.x + mForward * std::cos( aHeading ) - mRight * std::sin( aHeading ),
        aPosition.y + mForward * std::sin( aHeading ) + mRight * std::cos( aHeading )
    };
    return DistanceTo( Location, aLine ) <= HalfLineWidth;
}

//-----------------------------------------------------------------------------
// Finds the shortest distance to a closed loop. This shared geometry supports
// both point-on-line sensing and circular-body collision checks.
//-----------------------------------------------------------------------------
float CLineSensor::DistanceTo( const Vec2D& aPoint, const std::vector<Vec2D>& aLoop )
{
    const float NoHit = 1000000.0f;
    float Closest = NoHit;

    // Consecutive vertices form segments; the modulo joins the last vertex to
    // the first. Project the point onto each segment and clamp the projection
    // so distances near corners are measured to the appropriate endpoint.
    for( std::size_t i = 0; i < aLoop.size(); ++i )
    {
        const Vec2D& Start = aLoop[i];
        const Vec2D& End = aLoop[(i + 1) % aLoop.size()];
        const float Dx = End.x - Start.x;
        const float Dy = End.y - Start.y;
        const float LengthSquared = Dx * Dx + Dy * Dy;
        float Fraction = 0.0f;
        if( LengthSquared > 0.0f )
        {
            Fraction = ((aPoint.x - Start.x) * Dx + (aPoint.y - Start.y) * Dy) / LengthSquared;
        }

        // A zero-length segment leaves Fraction at zero and is safely treated
        // as a single point rather than causing division by zero.
        if( Fraction < 0.0f )
        {
            Fraction = 0.0f;
        }
        if( Fraction > 1.0f )
        {
            Fraction = 1.0f;
        }
        const float OffsetX = aPoint.x - (Start.x + Fraction * Dx);
        const float OffsetY = aPoint.y - (Start.y + Fraction * Dy);
        const float Distance = std::sqrt( OffsetX * OffsetX + OffsetY * OffsetY );
        if( Distance < Closest )
        {
            Closest = Distance;
        }
    }
    return Closest;
}
