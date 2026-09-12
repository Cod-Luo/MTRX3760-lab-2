// CLineRobot.h - A2's two binary line sensors with shared noisy motion.
#ifndef CLINEROBOT_H
#define CLINEROBOT_H
#include "CLineSensor.h"
#include "CNoisyMotion.h"

class CLineRobot
{
    public:
        CLineRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed );
        void Update( const std::vector<Vec2D>& aLine );
        const CNoisyMotion& GetMotion() const;
    private:
        CNoisyMotion mMotion;
        CLineSensor mCentreSensor;
        CLineSensor mSideSensor;
};
#endif
