#ifndef DRIVETRAIN_H
#define DRIVETRIAN_H

#include <stdint.h>
#include <stdbool.h>

//Struct for the return value of driveTrain_calc()
typedef struct DrivetrainOut{
    double   ratio; //best ratio found
    uint16_t* front; //ptr to front gear for best ratio
    uint16_t* rear; //ptr to rear gear for best ratio
}DrivetrainOut_t;

/*drivetrain_calc()
 * function finds closet ratio to the goal without going over
 * WARN: uses heap
 * WARN: uses recursion
 * RETURN: false if no ratio could be found (ALL BUSTS)
 * PARAMS:
 *    double* const targetRatio => Goal
 *    uint16_t* frontBuff => Front gear array; must be sorted descending; no repeats
 *    uint8_t frontLen => length of frontBuff
 *    uint16_t* rearBuff => Rear gear array; must be sorted descending; no repeats
 *    uint8_t rearLen => length of rearBuff
 *    DrivetrainOut* out => ptr to output struct
 *    */
bool drivetrain_calc(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        DrivetrainOut_t* out);

/*drivetrain_shift()
 * prints the shifting sequence to reach target ratio.
 * if no valid ration is found will print:
 * ```Target Ratio Could not be found
 * END```
 * WARN: uses heap
 * WARN: uses recursion
 * RETURN: best ratio found without exceeding the goal
 * PARAMS:
 *    double* const targetRatio => Goal
 *    uint16_t* frontBuff => Front gear array; must be sorted descending; no repeats
 *    uint8_t frontLen => length of frontBuff
 *    uint16_t* rearBuff => Rear gear array; must be sorted descending; no repeats
 *    uint8_t rearLen => length of rearBuff
 *    uint16_t frontPos => ptr to starting position of front gear in frontBuff
 *       will be modified to final position
 *    uint16_t rearPos => ptr to starting position of rear gear in rearBuff
 *       will be modified to final position
 *    */
double drivetrain_shift(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        uint16_t* frontPos, uint16_t* rearPos);
#endif
