/* Matthew Tuer 
november 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
*/
#ifndef LIFTDYNAMICS_H
#define LIFTDYNAMICS_H
#include <stdint.h>

#include "freertos/FreeRTOS.h"




typedef struct {
    
    
    int cOfG;
    int cOfGHeight;
    int FwToLoad;
    float maxFlatLoad;
    float wheelBase;
    float weightKG;
    float maxDynamicLoad;
    float maxAcceleration;
    float safetyFactor;
    float loadScore;
    float brakeScore; 

   
} ForkLift;

ForkLift* initializeLift();

bool calculateDynamicLoad(ForkLift*,int,int);
bool calculateMaxDeaccel(ForkLift*,float,float,float);

bool calculateSafetyScores(ForkLift*,float,float);


#endif 
