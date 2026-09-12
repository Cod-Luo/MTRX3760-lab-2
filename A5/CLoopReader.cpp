//-----------------------------------------------------------------------------
// CLoopReader.cpp
//
// Reads a loop file into a single loop. The file format is described at the top
// of CLoopReader.h. ReadFile ignores blank lines and '#' comments, and stops
// with a clear message naming the line if it meets something it cannot parse.
//-----------------------------------------------------------------------------

#include "CLoopReader.h"

#include <fstream>
#include <iostream>
#include <sstream>

//---Degrees in the file are converted to radians for use in the program.------
const float CLoopReader::DegreesToRadians = 3.14159265f / 180.0f;


//-----------------------------------------------------------------------------
CLoopReader::CLoopReader()
    :
        mStartPosition( { 0.0f, 0.0f } ),
        mStartHeading( 0.0f )
{
}


//-----------------------------------------------------------------------------
const Vec2D& CLoopReader::GetStartPosition() const
{
    return mStartPosition;
}


//-----------------------------------------------------------------------------
float CLoopReader::GetStartHeading() const
{
    return mStartHeading;
}


//-----------------------------------------------------------------------------
const std::vector<Vec2D>& CLoopReader::GetVertices() const
{
    return mVertices;
}


//-----------------------------------------------------------------------------
bool CLoopReader::ReadFile( const std::string& arFilename )
{
    // A reader can be reused without retaining data from the previous map.
    mStartPosition = Vec2D{ 0.0f, 0.0f };
    mStartHeading = 0.0f;
    mVertices.clear();

    bool Okay = true;

    std::ifstream File( arFilename );
    if( !File )
    {
        std::cout << "CLoopReader: could not open file '" << arFilename << "'" << std::endl;
        Okay = false;
    }

    std::string Line;
    int LineNumber = 0;
    bool HaveLoop = false;
    bool HaveStartPose = false;
    std::string LoopName;

    while( Okay && std::getline( File, Line ) )
    {
        ++LineNumber;

        // Strip any comment: everything from a '#' to the end of the line.
        std::string::size_type Hash = Line.find( '#' );
        if( Hash != std::string::npos )
        {
            Line = Line.substr( 0, Hash );
        }

        // Split the remaining line into whitespace-separated words. A line with
        // no words (blank, or comment-only) has no keyword and is skipped.
        std::istringstream Words( Line );
        std::string Keyword;
        if( Words >> Keyword )
        {
            if( Keyword == "loop" )
            {
                if( HaveLoop )
                {
                    std::cout << "CLoopReader: a second 'loop' on line " << LineNumber
                              << " (a file describes one loop)" << std::endl;
                    Okay = false;
                }
                else if( !(Words >> LoopName) )
                {
                    std::cout << "CLoopReader: 'loop' needs a name on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else
                {
                    HaveLoop = true;
                }
            }
            else if( Keyword == "startpose" )
            {
                float X = 0.0f;
                float Y = 0.0f;
                float HeadingDegrees = 0.0f;
                if( HaveStartPose )
                {
                    std::cout << "CLoopReader: a second 'startpose' on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else if( !(Words >> X >> Y >> HeadingDegrees) )
                {
                    std::cout << "CLoopReader: 'startpose' needs x, y and heading on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else if( !HaveLoop )
                {
                    std::cout << "CLoopReader: 'startpose' before any 'loop' on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else
                {
                    mStartPosition = Vec2D{ X, Y };
                    mStartHeading = HeadingDegrees * DegreesToRadians;
                    HaveStartPose = true;
                }
            }
            else if( Keyword == "vertex" )
            {
                float X = 0.0f;
                float Y = 0.0f;
                if( !(Words >> X >> Y) )
                {
                    std::cout << "CLoopReader: 'vertex' needs x and y on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else if( !HaveLoop )
                {
                    std::cout << "CLoopReader: 'vertex' before any 'loop' on line "
                              << LineNumber << std::endl;
                    Okay = false;
                }
                else
                {
                    mVertices.push_back( { X, Y } );
                }
            }
            else
            {
                std::cout << "CLoopReader: unknown keyword '" << Keyword << "' on line "
                          << LineNumber << std::endl;
                Okay = false;
            }
        }
    }

    if( Okay && (!HaveLoop || !HaveStartPose || mVertices.size() < 3) )
    {
        std::cout << "CLoopReader: file needs one loop, a startpose and at least three vertices"
                  << std::endl;
        Okay = false;
    }

    return Okay;
}
