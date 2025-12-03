/* Matthew Tuer 
november 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
*/

#include "liftDynamics.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include <math.h>
ForkLift* initializeLift(){
    ForkLift *lift_instance = malloc(sizeof(ForkLift));
    lift_instance->wheelBase=14.5;
    lift_instance->weightKG=1.360; //=0.766;
    lift_instance->cOfG=9.5;
    lift_instance->FwToLoad=10;
    lift_instance->maxDynamicLoad=0;
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