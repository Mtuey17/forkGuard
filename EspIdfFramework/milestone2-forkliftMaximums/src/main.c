#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "VL53L.h"
#include "IMU.h"
#include "liftDynamics.h"
#include "straingauge.h"
#include "OLED.h"

#include <font_latin_8x8.h>
#include <ssd1306.h>
#include <bitmap_icon.h>
#include <bdf_font_emoticon_22x21.h>
#include <bdf_font_nenr12_21x26.h>
#include "string.h"
#include <stdio.h>
#include <math.h>
#include "wirelessSetup.h"


#define WIFI_SSID "ForkGuardNet"
#define WIFI_PASS "Guard1234"

void app_main() {

initWifi(WIFI_SSID,WIFI_PASS);
esp_mqtt_client_handle_t mqttClient=initMQTT();



//i2c 0 for IMU and OLED----------------------------------
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

//i2c 1 for TOF -----------------------------------------
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


    //Strain Amp setup
    #define RSTX 36
    #define RSRX 38
    HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2,1.29);
    HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1,1.29);
    bool flipStrainRead=false;


    //sensors have matching ids, two seperate i2c busses needed
    //will fix later 
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();

    //OLED init (using same I2C bus as IMU)
    ssd1306_handle_t OLED=initializeOLED(bus_handle1);//initialize OLED
    int oledCount=0;

    while(1){


        if (flipStrainRead){HX711_updateWeight(leftForkSensor, 20);}
        else{HX711_updateWeight(rightForkSensor, 20);}
        flipStrainRead=!flipStrainRead;
        
    
        //updateDistance(TOF);
        vTaskDelay(pdMS_TO_TICKS(10));
        updateAngles(IMU);
        calculateDynamicLoad(toyLift,IMU->pitch,9.5);
        
 
        //ESP_LOGI("main","leftW: %d|rightW: %d",leftForkSensor->Weight,rightForkSensor->Weight);


        ESP_LOGI("main","pitch: %d | maxLoad: %f kg| currentLoad: %f kg",IMU->pitch,toyLift->maxDynamicLoad,rightForkSensor->kgWeight*-2);
        vTaskDelay(pdMS_TO_TICKS(50));

        oledCount++;
        if (oledCount==10){

        
        char toDisplay[32];
        snprintf(toDisplay, sizeof(toDisplay), "m: %0.3f", toyLift->maxDynamicLoad);
        ssd1306_display_text_x2(OLED, 0, toDisplay, false);
        char toDisplay2[32];
        snprintf(toDisplay2, sizeof(toDisplay2), "c: %0.3f", fabsf(rightForkSensor->kgWeight*-2.0f));
        ssd1306_display_text_x2(OLED, 2, toDisplay2, false);
     

        char msg[128];
        snprintf(msg, sizeof(msg), "Driver:Matt,MaxWeight:%0.3f,CurrentWeight:%0.3f,Pitch:%d,:LoadHeight:%d", toyLift->maxDynamicLoad,fabsf(rightForkSensor->kgWeight*-2.0f),IMU->pitch,10);
        esp_mqtt_client_publish(mqttClient, "toyLift1/sensors", msg, 0, 1, 0);
        oledCount=0;
        }

        
 
        
        
    }


}