#include "drivetrain.h"


typedef struct DrivetrainMeta{
    double bestRatio;
    uint16_t* bestOuterElm;
    uint16_t* bestInnerElm;
    /* enum skipped_t{SKIPPED, NOT_SKIPPED} isSkipped; */
}DrivetrainMeta_t;

void checkRatio(){}

DrivetrainOut_t calc_drivetrain(const double targetRatio,
       const uint16_t* frontBuff, const uint8_t frontLen,
       const uint16_t* rearBuff, const uint8_t rearLen){
    //=Get starting best=================================
    DrivetrainMeta_t best = {(frontBuff[0]/ rearBuff[0]), frontBuff, rearBuff};
    //=Determine longer==================================
    //--FIXME:SKIP for now
    //=Loop on longer====================================

    for(uint8_t i = 0; i < frontLen; i++){
        //=Test bookends=================================
        //=loop up or down=======================
    }
    //find champ
}
