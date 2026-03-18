/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 

*/
#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "driver/i2c_master.h"  
#include <ssd1306.h>
#include "liftDynamics.h"



ssd1306_handle_t initializeOLED(i2c_master_bus_handle_t);
void drawWeightChart(ssd1306_handle_t ,float,float);
void drawBalanceChart(ssd1306_handle_t,float);
void showWeightInfo(ssd1306_handle_t,uint32_t*,ForkLift*);

bool showDriver(ssd1306_handle_t,uint32_t*,const char*[],ForkLift*);



extern const uint8_t forklift_guard_icon_128x32[];

#endif 
