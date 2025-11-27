/* Matthew Tuer 
november  16th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 

Sprint 2-Demo
this demo validates:
operation of IMU
operation of strain gauges
operation of oled
dynamic weight estimation formula 

this program displays the maximum load the toy lift can carry without tipping
the current load on the forks will also be displayed on OLED
all sensor data is sent to an MQTT server where it is displayed on a basic web UI
*/



#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <font_latin_8x8.h>
#include <ssd1306.h>
#include <bitmap_icon.h>
#include <bdf_font_emoticon_22x21.h>
#include <bdf_font_nenr12_21x26.h>
#include "string.h"
#include <stdio.h>
#include <math.h>

//my custom made libraries 
#include "wirelessSetup.h"
#include "VL53L.h"
#include "IMU.h"
#include "liftDynamics.h"
#include "straingauge.h"
#include "OLED.h"


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
    HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2,1.74626);
    HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1,1.569);
    bool flipStrainRead=false;


    //sensors have matching ids, two seperate i2c busses needed
    //will fix later 
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();

    //OLED init (using same I2C bus as IMU)
    ssd1306_handle_t OLED=initializeOLED(bus_handle1);//initialize OLED
    bool demoUI=false;

    if (demoUI){
    drawWeightChart(OLED,0.3,0.5);
    }
    else{


    int oledCount=0;

    while(1){

        //flipping what strain gauge is being read every loop 
        if (flipStrainRead){HX711_updateWeight(leftForkSensor, 40);}
        else{HX711_updateWeight(rightForkSensor, 40);}
        flipStrainRead=!flipStrainRead;
        
        //updateDistance(TOF); //working, but not used due to physical limitations of toy lift 
        vTaskDelay(pdMS_TO_TICKS(10));
        updateAngles(IMU);
        calculateDynamicLoad(toyLift,IMU->pitch,11.5);//calculating max load at current pitch/load height
        float currentWeight=(rightForkSensor->kgWeight*-1.0)+(leftForkSensor->kgWeight*-1.0);


        //ESP_LOGI("main"," Left raw: %d || Right raw: %d",leftForkSensor->Weight,rightForkSensor->Weight);

        ESP_LOGI("main","pitch: %d | maxLoad: %f kg| currentLoad: %f kg",IMU->pitch,toyLift->maxDynamicLoad,currentWeight);
        vTaskDelay(pdMS_TO_TICKS(50));

        oledCount++;
        if (oledCount==10){//displaying values on oled 
        char toDisplay[32];
        snprintf(toDisplay, sizeof(toDisplay), "m: %0.3f", toyLift->maxDynamicLoad);
        ssd1306_display_text_x2(OLED, 0, toDisplay, false);
        char toDisplay2[32];
        
        snprintf(toDisplay2, sizeof(toDisplay2), "c: %0.3f", fabsf(currentWeight));
        ssd1306_display_text_x2(OLED, 2, toDisplay2, false);
     
        //calculating safety score 
        int safetyScore=100;
        float threshold85 = toyLift->maxDynamicLoad * 0.85f;
        if (fabsf(rightForkSensor->kgWeight*-2.0f) > threshold85) {
        float span =  toyLift->maxDynamicLoad - threshold85;  
        float excess = fabsf(rightForkSensor->kgWeight*-2.0f)  - threshold85;
        // Linear drop from 100 → 0
        float fraction = excess / span;  
        safetyScore = (int)(100.0f * (1.0f - fraction));
        }
        //ESP_LOGI("main","safetyScore: %d",safetyScore);


        //sending to MQTT
        char msg[128];
        snprintf(msg, sizeof(msg), "Driver:Matt,MaxWeight:%0.3f,CurrentWeight:%0.3f,Pitch:%d,LoadHeight:%d,score:%d", toyLift->maxDynamicLoad,currentWeight,IMU->pitch,10,safetyScore);
        
        esp_mqtt_client_publish(mqttClient, "toyLift1/sensors", msg, 0, 1, 0);
        oledCount=0;
        }    
    }
}


}