/* Matthew Tuer 
november  16th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 


*/


#include "driver/gpio.h"
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
#include "driver/i2s_std.h"

#include "mfrc522.h"
#include "microphone.h"
//my custom made libraries 
#include "wirelessSetup.h"
#include "VL53L.h"
#include "IMU.h"
#include "liftDynamics.h"
#include "straingauge.h"
#include "OLED.h"



void app_main() {

    //------------DEFINITIONS, BUS INIT, WIFI/MQTT INIT------------
    #define WIFI_SSID "ForkGuardNet"
    #define WIFI_PASS "Guard1234"
    initWifi(WIFI_SSID,WIFI_PASS);
    MQTTHandler* MQTT=initMQTT();
    #define SDA1PIN 40
    #define SCL1PIN 41
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
    #define SDA2PIN 15
    #define SCL2PIN 16
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

    //I2S init for mic  
    #define PIN_SCK GPIO_NUM_17
    #define PIN_WS  GPIO_NUM_18
    #define PIN_SD  GPIO_NUM_8

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_chan_handle_t I2S_Handle;
    i2s_new_channel(&channel_config, NULL, &I2S_Handle);
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_SCK,
            .ws   = PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_SD,
            .invert_flags = { 0 },
        },
    };
    i2s_channel_init_std_mode(I2S_Handle, &std_cfg);
    i2s_channel_enable(I2S_Handle);
    
    //------------DEFINITIONS, BUS INIT, WIFI/MQTT INIT------------

  
    //--------------------SENSOR INIT--------------------
    //Strain Amp setup
    #define RSTX 20
    #define RSRX 19
    HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2,2.29);//1.74626
    HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1,1.32);
    bool flipStrainRead=false; 
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();
    ssd1306_handle_t OLED=initializeOLED(bus_handle1);//OLED init (using same I2C bus as IMU)
    ICS43434* microphone=InitICS(I2S_Handle);
    mfrc522_init();
    //--------------------SENSOR INIT--------------------
   

    //---------------------GPIO INIT---------------------
    #define GAS_SENSOR_PIN GPIO_NUM_35
    #define GAS_WARNING_LED GPIO_NUM_47
    gpio_set_direction(GAS_SENSOR_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(GAS_WARNING_LED,GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_48,GPIO_MODE_OUTPUT);//weight warning LED
    //---------------------GPIO INIT---------------------

    
    
    while(1){


        //----------------DATA AQUISITION----------------
        mfrc522_read_uid(toyLift->currentDriver);
        //flipping what strain gauge is being read every loop 
        if (flipStrainRead){HX711_updateWeight(leftForkSensor, 40);}
        else{HX711_updateWeight(rightForkSensor, 40);}
        flipStrainRead=!flipStrainRead;
        updateDistance(TOF); 
        updateAngles(IMU);
        updateAcceleration(IMU);
        toyLift->gas=gpio_get_level(GAS_SENSOR_PIN);
        gpio_set_level(GAS_WARNING_LED,!toyLift->gas);
        checkMicLevel(microphone);
        toyLift->noise=microphone->dbHigh;
        //----------------DATA AQUISITION----------------


        //---------------DATA MANIPULATION---------------
        toyLift->currentAccel=IMU->rawYAcceleration;
        toyLift->currentLoad=(leftForkSensor->kgWeight*-1.0)*2.0;
        if (toyLift->currentLoad<0.0){
            toyLift->currentLoad=0.0;
        }
       calculateDynamicLoad(toyLift,IMU->pitch,TOF->distance);//calculating max load at current pitch/load height
       calculateMaxDeaccel(toyLift,toyLift->currentLoad, TOF->distance, IMU->pitch);
       calculateSafetyScores(toyLift,toyLift->currentLoad,IMU->rawYAcceleration);
        //---------------DATA MANIPULATION---------------


        //------------OUTPUTS AND NETWORKING-------------
        drawWeightChart(OLED,toyLift->currentLoad,toyLift->maxDynamicLoad);
        sendMQTT(MQTT,toyLift);
        ESP_LOGI("main","maxAccel: %f | currentAccel: %f",toyLift->maxAcceleration,toyLift->currentAccel);
        ESP_LOGI("main","pitch: %d | maxLoad: %f kg| currentLoad: %f kg| height: %dcm|GAS:%d|UID:%02X%02X%02X%02X",IMU->pitch,toyLift->maxDynamicLoad,toyLift->currentLoad,TOF->distance,toyLift->gas,toyLift->currentDriver[0], toyLift->currentDriver[1], toyLift->currentDriver[2], toyLift->currentDriver[3]);
        //------------OUTPUTS AND NETWORKING-------------


        vTaskDelay(pdMS_TO_TICKS(40));
    }
}


