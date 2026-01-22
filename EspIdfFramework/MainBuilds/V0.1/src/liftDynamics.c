/* Matthew Tuer 
november 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
*/

#include "liftDynamics.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
ForkLift* initializeLift(){
    ForkLift *lift_instance = malloc(sizeof(ForkLift));
    lift_instance->wheelBase=14.5;
    lift_instance->weightKG=1.360; //=0.766;
    lift_instance->cOfG=9.5;
    lift_instance->cOfGHeight=5.95;
    lift_instance->FwToLoad=10;
    lift_instance->maxDynamicLoad=0;
    lift_instance->maxAcceleration=0;
    lift_instance->safetyFactor=.8f;
    lift_instance->loadScore=100.0f;
    lift_instance->brakeScore=100.0f;
    lift_instance-> currentLoad=0;
    lift_instance-> currentAccel=0;
    lift_instance-> gas=0;
    lift_instance-> noise=0;
    memset(lift_instance->currentDriver, 0, sizeof(lift_instance->currentDriver));
    memset(lift_instance->previousDriver, 0, sizeof(lift_instance->previousDriver));

    
    lift_instance->maxFlatLoad= (float)(lift_instance->weightKG*( (lift_instance->wheelBase-lift_instance->cOfG)/lift_instance->FwToLoad));
    return lift_instance; 
}


bool calculateDynamicLoad(ForkLift* FL, int pitch, int Lh){

    float radPitch=pitch*(M_PI / 180.0);
    float topTerm=  ( FL->wheelBase- (FL->cOfG*cos(radPitch)) )+ (Lh*sin(radPitch));
    float bottomTerm=(FL->FwToLoad*cos(radPitch))-(5*sin(radPitch));
    FL->maxDynamicLoad=FL->weightKG*(topTerm/bottomTerm);
    return true;

}

bool calculateMaxDeaccel(ForkLift* FL,float loadWeight, float forkHeight, float pitch){


    float assumedLoadHeight=5;
    float Hl=forkHeight+assumedLoadHeight/2;//height of loads CofG

    float Heq=((FL->weightKG * FL->cOfGHeight) +(loadWeight * Hl)) /(FL->weightKG + loadWeight);//combined CofG height 
    if (Heq==0.0){return false;}
    


    
    // Convert pitch to radians
    float radPitch = pitch * (M_PI / 180.0f);

    //accounting for pitch 
    float xf = (FL->wheelBase - FL->cOfG) * cosf(radPitch)+ FL->cOfGHeight * sinf(radPitch);
    float dload = FL->FwToLoad * cosf(radPitch) + Hl * sinf(radPitch); // front axle to load CG


    if ((FL->weightKG * xf) <= (loadWeight * dload))//checking if lift already tipping 
    {
        FL->maxAcceleration = 0.0f;
        return false;
    }
    float Heq_eff = Heq * cosf(radPitch);
    FL->maxAcceleration = 9.81f * ((FL->weightKG * xf) - (loadWeight * dload)) / ((FL->weightKG + loadWeight) * Heq_eff);// Max safe deceleration
    return true;
}

bool calculateSafetyScores(ForkLift* FL,float currentLoad,float currentAccel){
    
    //if load/deaccel is under  safetyFactor precent of maximums then safety score will remain 100
    //if load/deaccel is within safetyFactor precent of maximums safety score will decrease untill it reaches
    //0 when load/deaccel is equal to its maximum 
    FL->loadScore=100.0f;
    FL->brakeScore=100.0f;
    float safetyThreshold = FL->maxDynamicLoad * FL->safetyFactor;//calculating load safety score 
        if (fabsf(currentLoad) > safetyThreshold) {
        float span =  FL->maxDynamicLoad - safetyThreshold;  
        float excess = fabsf(currentLoad)  - safetyThreshold;
        float fraction = excess / span;  
        FL->loadScore = (int)(100.0f * (1.0f - fraction));
        }
    
    safetyThreshold = FL->maxAcceleration * FL->safetyFactor;//calculating accel safety score 
        if (fabsf(currentAccel) > safetyThreshold) {
        float span =  FL->maxAcceleration - safetyThreshold;  
        float excess = fabsf(currentAccel)  - safetyThreshold;
        float fraction = excess / span;  
        FL->brakeScore = (int)(100.0f * (1.0f - fraction));
        }
    return true;
}