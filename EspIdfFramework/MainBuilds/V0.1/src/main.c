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


#define WIFI_SSID "ForkGuardNet"
#define WIFI_PASS "Guard1234"
initWifi(WIFI_SSID,WIFI_PASS);
MQTTHandler* MQTT=initMQTT();



//i2c 0 for IMU and OLED----------------------------------
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

//------------------------------------------------------


//i2c 1 for TOF -----------------------------------------
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

//-----------------------------------------------------


  

    //Strain Amp setup
    #define RSTX 20
    #define RSRX 19
    HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2,2.29);//1.74626
    HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1,1.32);
    bool flipStrainRead=false;

    

    //sensors have matching ids, two seperate i2c busses needed
    //will fix later 
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();

    //OLED init (using same I2C bus as IMU)
    ssd1306_handle_t OLED=initializeOLED(bus_handle1);//initialize OLED
    

    char toDisplay[32];
    snprintf(toDisplay, sizeof(toDisplay), "F.G--M.T");
    ssd1306_display_text_x2(OLED, 0, toDisplay, false);
    ssd1306_display_text_x2(OLED, 2, toDisplay, false);

    
   
    #define GAS_SENSOR_PIN GPIO_NUM_35
    #define GAS_WARNING_LED GPIO_NUM_47
    gpio_set_direction(GAS_SENSOR_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(GAS_WARNING_LED,GPIO_MODE_OUTPUT);

    gpio_set_direction(GPIO_NUM_48,GPIO_MODE_OUTPUT);


    //microphoneInit();
    mfrc522_init();
    
    while(1){


        mfrc522_read_uid(toyLift->currentDriver);

        //flipping what strain gauge is being read every loop 
        if (flipStrainRead){HX711_updateWeight(leftForkSensor, 40);}
        else{HX711_updateWeight(rightForkSensor, 40);}
        flipStrainRead=!flipStrainRead;
       

        updateDistance(TOF); 
        updateAngles(IMU);
        toyLift->currentAccel=IMU->rawYAcceleration;
        updateAcceleration(IMU);

        toyLift->currentLoad=(leftForkSensor->kgWeight*-1.0)*2.0;
        if (toyLift->currentLoad<0.0){
            toyLift->currentLoad=0.0;
        }

        //checking advanced metrics 
       calculateDynamicLoad(toyLift,IMU->pitch,TOF->distance);//calculating max load at current pitch/load height
       calculateMaxDeaccel(toyLift,toyLift->currentLoad, TOF->distance, IMU->pitch);
       calculateSafetyScores(toyLift,toyLift->currentLoad,IMU->rawYAcceleration);
    
        //checking gas and noise levels 
        toyLift->gas=gpio_get_level(GAS_SENSOR_PIN);
        gpio_set_level(GAS_WARNING_LED,!toyLift->gas);

        drawWeightChart(OLED,toyLift->currentLoad,toyLift->maxDynamicLoad);


        sendMQTT(MQTT,toyLift);

        //ESP_LOGI("main","maxAccel: %f | currentAccel: %f",toyLift->maxAcceleration,toyLift->currentAccel);
        //ESP_LOGI("main","pitch: %d | maxLoad: %f kg| currentLoad: %f kg| height: %dcm|GAS:%d|UID:%02X%02X%02X%02X",IMU->pitch,toyLift->maxDynamicLoad,toyLift->currentLoad,TOF->distance,gasLevelOK,toyLift->currentDriver[0], toyLift->currentDriver[1], toyLift->currentDriver[2], toyLift->currentDriver[3]);

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}


