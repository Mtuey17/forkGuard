/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
Header file for BNO055 IMU
*/
#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "driver/i2c_master.h"  
#include <ssd1306.h>



ssd1306_handle_t initializeOLED(i2c_master_bus_handle_t);
void drawWeightChart(ssd1306_handle_t ,float,float);

extern const uint8_t exclaim_bitmap8x16[];
extern const uint8_t emptyline_icon_32x32[];
extern const uint8_t whiteLine_icon_32x32[];

#endif 
