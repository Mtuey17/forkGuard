#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "VL53L.h"
#include "IMU.h"
#include "liftDynamics.h"
#include "straingauge.h"


#include <font_latin_8x8.h>
#include <ssd1306.h>
#include <bitmap_icon.h>
#include <bdf_font_emoticon_22x21.h>
#include <bdf_font_nenr12_21x26.h>

void app_main() {



//i2c 0 for IMU----------------------------------
#define SDA1PIN 16
#define SCL1PIN 15
i2c_master_bus_config_t I2C_0_CONFIG = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCL1PIN,
    .sda_io_num = SDA1PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle1;
i2c_new_master_bus(&I2C_0_CONFIG, &bus_handle1);
//------------------------------------------------------

//i2c 1 for TOF-----------------------------------------
#define SDA2PIN 12
#define SCL2PIN 13
i2c_master_bus_config_t I2C_1_CONFIG = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_1,
    .scl_io_num = SCL2PIN,
    .sda_io_num = SDA2PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle2;
i2c_new_master_bus(&I2C_1_CONFIG, &bus_handle2);
//-----------------------------------------------------


        
    #define RSTX 20
    #define RSRX 21
    //HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1);
    //HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2);
    bool flipStrainRead=false;


    //sensors have matching ids, two seperate i2c busses needed
    //will fix later 
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();




    while(1){


        //if (flipStrainRead){HX711_updateWeight(leftForkSensor, 20);}
        //else{HX711_updateWeight(rightForkSensor, 20);}
        flipStrainRead=!flipStrainRead;
    
        //updateDistance(TOF);
        vTaskDelay(pdMS_TO_TICKS(10));
        updateAngles(IMU);
        calculateDynamicLoad(toyLift,IMU->pitch,TOF->distance);
        //IMUInfo(IMU);

  
        ESP_LOGI("main","pitch: %d|loadHeight: %d|maxLoad: %f kg",IMU->pitch,TOF->distance,toyLift->maxDynamicLoad);
        vTaskDelay(pdMS_TO_TICKS(50));


        
 
        
        
    }


}