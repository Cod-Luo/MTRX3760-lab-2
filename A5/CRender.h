// CRender.h - All raylib access and colour conversion stay behind this interface.
#ifndef CRENDER_H
#define CRENDER_H

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
        CRender();
        ~CRender();
        bool WindowShouldClose() const;
        void BeginDrawing();
        void EndDrawing();
        void DrawCircle( Vec2D aPosition, int aRadius, Colour aColour );
        void DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, Colour aColour );
    private:
        CRender( const CRender& ) = delete;
        CRender& operator=( const CRender& ) = delete;
};
#endif
