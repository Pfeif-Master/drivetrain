#ifndef DRIVETRAIN_H
#define DRIVETRIAN_H

#include <stdint.h>
typedef struct DrivetrainOut{
    const double   ratio;
    const uint16_t front;
    const uint16_t rear;
}DrivetrainOut_t;

DrivetrainOut_t calc_drivetrain(const double targetRatio,
       const uint16_t* frontBuff, const uint8_t frontLen,
       const uint16_t* rearBuff, const uint8_t rearLen);
#endif
