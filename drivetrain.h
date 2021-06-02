#ifndef DRIVETRAIN_H
#define DRIVETRIAN_H

#include <stdint.h>
#include <stdbool.h>

typedef struct DrivetrainOut{
    double   ratio;
    uint16_t* front;
    uint16_t* rear;
}DrivetrainOut_t;

bool drivetrain_calc(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        DrivetrainOut_t* out);

double drivetrain_shift(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        uint16_t* const frontPos, uint16_t* const rearPos);
#endif
