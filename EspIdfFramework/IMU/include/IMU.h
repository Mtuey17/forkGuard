/* Matthew Tuer, July 30, 2025 mtuer@uwaterloo.ca/matthewjtuer@gmail.com */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "driver/i2c_master.h"  

typedef struct {
    i2c_master_dev_handle_t dev_handle;    
    uint8_t deviceID;         
    int xAngle;    
    int yAngle;     
    int zAngle;                
   
} IMU_t;

IMU_t *initializeIMU(i2c_master_bus_handle_t,uint8_t);

void updateAngles(IMU_t*);



#endif 
