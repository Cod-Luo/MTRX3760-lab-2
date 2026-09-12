// CRobot.h - A2's two-range-sensor controller with shared noisy motion.
#ifndef CROBOT_H
#define CROBOT_H
#include "CSensor.h"
#include "CNoisyMotion.h"

class CRobot
{
    public:
        CRobot( const Vec2D& aStart, float aHeading, unsigned int aSeed );
        void Update( const std::vector<Vec2D>& aWalls );
        const CNoisyMotion& GetMotion() const;
    private:
        CNoisyMotion mMotion;
        CSensor mSensor90;
        CSensor mSensor45;
};
#endif
