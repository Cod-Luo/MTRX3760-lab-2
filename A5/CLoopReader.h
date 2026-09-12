//-----------------------------------------------------------------------------
// CLoopReader.h
//
// Reads a loop file and stores the single closed loop it describes: a starting
// pose and a list of vertices. A loop file describes a shape only;
// it knows nothing about rooms, lines or robots. SimpleWalls.map and
// SimpleLine.map are two example files.
//
//
// FILE FORMAT
//
// One instruction per line. The first word says what the line is; the rest
// are its values. Blank lines are ignored. Anything from a '#' to the end of a
// line is a comment and is ignored. Words are separated by spaces. A file
// describes one loop.
//
//   loop <name>              Name the loop. One per file.
//   startpose <x> <y> <deg>  The loop's starting pose: a point on or near the
//                            loop and a heading.
//   vertex <x> <y>           One corner of the loop. The last vertex is
//                            understood to join back to the first.
//
//
// COORDINATES
//
// Positions are in the space the renderer draws in: x runs left to right, y
// runs top to bottom, in an 800 x 600 window. Headings are in degrees,
// measured clockwise from the positive x axis (0 right, 90 down, 180 left,
// 270 up), and are converted to radians as the file is read.
//
//
// This is a starting point. Construct a reader, call ReadFile to load a file,
// or adapt it to suit your own design.
//-----------------------------------------------------------------------------

#ifndef CLOOPREADER_H
#define CLOOPREADER_H

#include "CRender.h"   // for Vec2D

#include <string>
#include <vector>

//-----------------------------------------------------------------------------
// CLoopReader: reads one loop file and stores the closed loop it describes.
//-----------------------------------------------------------------------------
class CLoopReader
{
    public:
        //---Ctor---
        CLoopReader();

        //---File reading---
        // Replaces any previously loaded loop. Returns false for an incomplete
        // file or a line that does not match the documented map format.
        bool ReadFile( const std::string& arFilename );

        //---Access to the loop that was read---
        const Vec2D& GetStartPosition() const;
        float GetStartHeading() const;
        const std::vector<Vec2D>& GetVertices() const;

    private:
        static const float DegreesToRadians;
        //---The loop---
        Vec2D mStartPosition;
        float mStartHeading;
        std::vector<Vec2D> mVertices;   // the loop's corners; last joins to first
};

#endif
