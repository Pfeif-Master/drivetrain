#include <stdio.h>
#include "drivetrain.h"

int main(){
    uint16_t front_cogs[2] = {38,30};
    uint16_t rear_cogs[4] = {28,23,19,16};
    double target_ratio = 1.6;

    shift(&target_ratio, front_cogs, 2, rear_cogs, 4, front_cogs, rear_cogs);

}
