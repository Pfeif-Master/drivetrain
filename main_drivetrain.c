#include <stdio.h>
#include "drivetrain.h"

int main(){
    /* const uint8_t FRONT_LEN = 2; */
    /* const uint8_t REAR_LEN = 4; */
    uint16_t front_cogs[2] = {38,30};
    uint16_t rear_cogs[4] = {28,23,19,16};
    double target_ratio = 1.6;

    DrivetrainOut_t out = calc_drivetrain(target_ratio, front_cogs, 2,
            rear_cogs, 4);

    printf("f:%d r: %d ratio: %f",out.front, out.rear, out.ratio);
}
