// CRender.cpp - Raylib window and drawing implementation.
#include "CRender.h"
#include "raylib.h"

#include <vector>

CRender::CRender()
{
    ::InitWindow( 800, 600, "MTRX3760 Lab 2 - A5 Noise Bonus" );
    ::SetTargetFPS( 60 );
}
CRender::~CRender() { ::CloseWindow(); }
bool CRender::WindowShouldClose() const { return ::WindowShouldClose(); }
void CRender::BeginDrawing()
{
    ::BeginDrawing();
    ::ClearBackground( BLACK );
}
void CRender::EndDrawing() { ::EndDrawing(); }
void CRender::DrawCircle( Vec2D aPosition, int aRadius, Colour aColour )
{
    ::DrawCircle( int( aPosition.x ), int( aPosition.y ), float( aRadius ),
                  Color{ aColour.r, aColour.g, aColour.b, aColour.a } );
}
void CRender::DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, Colour aColour )
{
    ::DrawLineEx( Vector2{ aStart.x, aStart.y }, Vector2{ aEnd.x, aEnd.y }, aThickness,
                  Color{ aColour.r, aColour.g, aColour.b, aColour.a } );
}

void CRender::DrawLineStrip( const std::vector<Vec2D>& aPoints, Colour aColour )
{
    if( aPoints.size() > 1 )
    {
        std::vector<Vector2> Points;
        Points.reserve( aPoints.size() );
        for( const Vec2D& Point : aPoints )
        {
            Points.push_back( Vector2{ Point.x, Point.y } );
        }
        ::DrawLineStrip( Points.data(), int( Points.size() ),
                         Color{ aColour.r, aColour.g, aColour.b, aColour.a } );
    }
}
