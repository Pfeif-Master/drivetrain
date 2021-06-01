#ifndef DRIVETRAIN_H
#define DRIVETRIAN_H

#include <stdint.h>

const double RATIO_MARGIN_OF_ERROR = 0.000001;
typedef struct DrivetrainOut{
    double   ratio;
    uint16_t front;
    uint16_t rear;
}DrivetrainOut_t;

DrivetrainOut_t calc_drivetrain(double targetRatio,
       uint16_t* frontBuff, uint8_t frontLen,
       uint16_t* rearBuff, uint8_t rearLen);
#endif
