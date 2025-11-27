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
    int FwToLoad;
    float maxFlatLoad;
    float wheelBase;
    float weightKG;
    float maxDynamicLoad;
   
} ForkLift;

ForkLift* initializeLift();

bool calculateDynamicLoad(ForkLift*,int,int);



#endif 
