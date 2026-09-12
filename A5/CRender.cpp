// CRender.cpp - Raylib window and drawing implementation.
#include "CRender.h"
#include "raylib.h"

CRender::CRender()
{
    ::InitWindow( 800, 600, "Robot Simulator" );
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
