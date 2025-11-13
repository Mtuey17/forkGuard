/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
Header file for BNO055 IMU
*/
#ifndef VL53L_H
#define VL53L_H

#include <stdint.h>
#include "driver/i2c_master.h"  

typedef struct {
    i2c_master_dev_handle_t dev_handle;    
    uint8_t deviceID;         
    int distance;
    int validID;
   
} VL53L_t;

VL53L_t *initializeTOF(i2c_master_bus_handle_t,uint8_t);
bool vl53l0x_set_address(VL53L_t*, uint8_t);
bool updateDistance(VL53L_t*);



#endif 
