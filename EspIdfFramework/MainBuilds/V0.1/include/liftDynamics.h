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
    float currentLoad;
    float maxAcceleration;
    float currentAccel;
    float safetyFactor;
    float loadScore;
    float brakeScore; 

    uint8_t currentDriver[4];
    uint8_t previousDriver[4];

    uint8_t gas;
    uint8_t noise;

   
} ForkLift;

ForkLift* initializeLift();

bool calculateDynamicLoad(ForkLift*,int,int);
bool calculateMaxDeaccel(ForkLift*,float,float,float);

bool calculateSafetyScores(ForkLift*,float,float);


#endif 
