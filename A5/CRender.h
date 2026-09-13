//-----------------------------------------------------------------------------
// CRender.h
//
// Declares the drawing interface used by A5. Raylib types and calls remain in
// CRender.cpp, leaving simulation classes dependent only on simple C++ types.
//-----------------------------------------------------------------------------
#ifndef CRENDER_H
#define CRENDER_H

#include <vector>

struct Vec2D
{
    float x;
    float y;
};

class CRender
{
    public:
        // Plain drawing data; no raylib types escape the wrapper.
        struct Colour
        {
            unsigned char r;
            unsigned char g;
            unsigned char b;
            unsigned char a;
        };

        // Opens and closes the single simulation window.
        CRender();
        ~CRender();

        // Reports window-close input and brackets one rendered frame.
        bool WindowShouldClose() const;
        void BeginDrawing();
        void EndDrawing();

        // Draws simulator geometry using wrapper-owned colours.
        void DrawCircle( Vec2D aPosition, int aRadius, Colour aColour );
        void DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, Colour aColour );
        // Draws every point in a trajectory as one connected path.
        void DrawLineStrip( const std::vector<Vec2D>& aPoints, Colour aColour );
    private:
        // A window owner cannot be copied safely. Private declarations prevent
        // copying using the copy-constructor syntax covered in Week 4.
        CRender( const CRender& );
        CRender& operator=( const CRender& );
};
#endif
