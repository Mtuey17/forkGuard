/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
Header file for BNO055 IMU
*/
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "driver/i2c_master.h"  

typedef struct {
    i2c_master_dev_handle_t dev_handle;    
    uint8_t deviceID;         
    int heading;    
    int roll;     
    int pitch;         
    float rawYAcceleration;
    float rawZAcceleration;
    float forwardAcceleration;
   
} IMU_t;

IMU_t *initializeIMU(i2c_master_bus_handle_t,uint8_t);

bool updateAngles(IMU_t*);

bool updateAcceleration(IMU_t*);

bool IMUInfo(IMU_t*);

#endif 
