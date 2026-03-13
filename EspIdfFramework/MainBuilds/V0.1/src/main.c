/* Matthew Tuer 
november  16th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 

need to add interupt pin from RTLS
when RTLS sends data, it will drive a pin high
this pin will invoke an interupt where a RTLS_READY flag will be set 
when RTLS_READY flag is set, main loop will read from RTLS uart next run
RLTS_READY flag will then be reset 

pin 9 looks good for interupt pin
*/


#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ssd1306.h>
#include <bitmap_icon.h>
#include "string.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "esp_timer.h"
#include "mfrc522.h"

//my custom made libraries 
#include "wirelessSetup.h"
#include "VL53L.h"
#include "IMU.h"
#include "liftDynamics.h"
#include "straingauge.h"
#include "OLED.h"
#include "RTLS.h"
#include "microphone.h"


volatile uint8_t UI_State=2;
volatile uint32_t lastPress=0;
volatile bool highLatch=false;
volatile bool previousLatchState=false;
volatile bool driverSelected;
volatile bool clearDisplay;
volatile uint32_t buttonPressStart = 0;
static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t currentTime=esp_timer_get_time()/1000;
    buttonPressStart=currentTime;
    if ((currentTime-lastPress)>200&&!highLatch&&driverSelected){     
    lastPress=currentTime;
    UI_State++;
    highLatch=true;
    if (UI_State>=4){UI_State=0;}
    if (UI_State==2){clearDisplay=true;}
    }
}

volatile bool RTLS_READY = false;
static void IRAM_ATTR rtls_isr_handler(void *arg)
{
RTLS_READY = true;
}


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
    i2s_chan_handle_t I2S_Handle=NULL;
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
    #define RTLS_TX 5
    #define RTLS_RX 6
    HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2,2.06);//1.74626
    HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1,1.38);
    bool flipStrainRead=false; 
    bool reCalibrate=false;
    int lastWeightTime=0;
    IMU_t *IMU=initializeIMU(bus_handle1,0x29);//imu init 
    VL53L_t *TOF=initializeTOF(bus_handle2,0x29);// tof init 
    ForkLift *toyLift= initializeLift();//not a physical sensor, but struct where all forklift parameters are saved together 
    ssd1306_handle_t OLED=initializeOLED(bus_handle1);//OLED init (using same I2C bus as IMU)
    uint32_t lastUpdate=0;
    uint32_t pressTime=0;
    ssd1306_clear_display(OLED,false);
    ICS43434* microphone=InitICS(I2S_Handle);//micrphone init 
    mfrc522_init();//rfid init 

    
     RTLS_Instance *RTLS=RTLS_init(UART_NUM_1,RTLS_RX,RTLS_TX,115200);//rtls init 
    //--------------------SENSOR INIT--------------------
   

    //---------------------GPIO INIT---------------------
    #define GAS_SENSOR_PIN GPIO_NUM_35
    #define GAS_WARNING_LED GPIO_NUM_47
    #define NOISE_WARNING_LED GPIO_NUM_21
    #define RTLS_INT_PIN GPIO_NUM_9
    #define UI_SELECTION_BUTTON GPIO_NUM_14

    gpio_set_direction(UI_SELECTION_BUTTON,GPIO_MODE_INPUT);
    gpio_set_pull_mode(UI_SELECTION_BUTTON, GPIO_PULLUP_ONLY); 
    gpio_set_direction(GAS_SENSOR_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(GAS_WARNING_LED,GPIO_MODE_OUTPUT);
    gpio_set_direction(NOISE_WARNING_LED,GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_48,GPIO_MODE_OUTPUT);//weight warning LED
    gpio_set_direction(RTLS_INT_PIN,GPIO_MODE_INPUT);//int pin for RTLS uart 
    gpio_set_direction(GPIO_NUM_45,GPIO_MODE_OUTPUT);//brake warning LED
    gpio_set_pull_mode(RTLS_INT_PIN, GPIO_PULLUP_ONLY); 

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_set_intr_type(UI_SELECTION_BUTTON, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(UI_SELECTION_BUTTON, button_isr_handler, NULL);
    gpio_set_intr_type(RTLS_INT_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(RTLS_INT_PIN, rtls_isr_handler, NULL);

    //---------------------GPIO INIT---------------------


    //---------------------MISC INIT---------------------

    //matching first byte of uid to a driver name (for UI)
    const char *driverNames[256] = {
    [(uint8_t)0x00] = "None",
    [(uint8_t)0xF9] = "Matthew",
    [(uint8_t)0x33] = "Bill .J",
    [(uint8_t)0xE3] = "Fred .M",
    };

    //---------------------MISC INIT---------------------

    //ssd1306_display_bitmap(OLED, 32, 0, forklift_guard_icon_64x32, 64, 32, false);


    while(1){
     


        uint32_t mainLoopStartTime=esp_timer_get_time()/1000;
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
        checkMicLevel(microphone);
        toyLift->noise=microphone->dbHigh;
        if (RTLS_READY){
        UPDATE_LOCATION(RTLS,20);
        RTLS_READY=false;
        }
        if (gpio_get_level(UI_SELECTION_BUTTON)==1){highLatch=false; pressTime=0;}
        else{pressTime = (esp_timer_get_time()/1000) - buttonPressStart;}
        if (pressTime>=3000){//if button is held for 3 seconds, reset strain amps 
            leftForkSensor->offsetCheck=false;
            rightForkSensor->offsetCheck=false;
        }

        //----------------DATA AQUISITION----------------


        //---------------DATA MANIPULATION---------------
        toyLift->currentAccel=IMU->rawYAcceleration;
        if (toyLift->currentAccel<toyLift->hardestBraketoday){toyLift->hardestBraketoday=toyLift->currentAccel;}//seeing if this is max for today (for UI)
        toyLift->currentLoad=(leftForkSensor->kgWeight+rightForkSensor->kgWeight);
        if (toyLift->currentLoad>toyLift->highestWeightToday){toyLift->highestWeightToday=toyLift->currentLoad;}//seeing if this is max for today (for UI)

        if (toyLift->currentLoad>0.05){
        toyLift->loadBalance=100.0f*((leftForkSensor->kgWeight-rightForkSensor->kgWeight)/leftForkSensor->kgWeight+rightForkSensor->kgWeight);
        }else{toyLift->loadBalance=0.0f;}
        
        if (toyLift->currentLoad>0.150){reCalibrate=true; lastWeightTime=esp_timer_get_time()/1000;}

        int currentTime=esp_timer_get_time()/1000;
  
        if (reCalibrate&&toyLift->currentLoad<0.149&&(currentTime-lastWeightTime)>500){
            reCalibrate=false;
            leftForkSensor->offsetCheck=false;
            rightForkSensor->offsetCheck=false;
        }
            
       calculateDynamicLoad(toyLift,IMU->pitch,TOF->distance);//calculating max load at current pitch/load height
       calculateMaxDeaccel(toyLift,toyLift->currentLoad, TOF->distance, IMU->pitch);
       calculateSafetyScores(toyLift,toyLift->currentLoad,IMU->rawYAcceleration);
        //---------------DATA MANIPULATION---------------


        //------------OUTPUTS AND NETWORKING-------------
        //ssd1306_display_text(OLED,1,"Hello Billy J!",false);
        gpio_set_level(NOISE_WARNING_LED,toyLift->noise);
        gpio_set_level(GAS_WARNING_LED,!toyLift->gas);
      
  
        
         //when driver changes,make ui show driver page
         if (memcmp(toyLift->currentDriver,toyLift->previousDriver,sizeof(toyLift->currentDriver)) != 0){UI_State=2;}
         if (clearDisplay){ssd1306_clear_display(OLED,false);clearDisplay=false;}
        switch ((UI_State))
        {
        case 0:
            showWeightInfo(OLED, &lastUpdate,toyLift);
            break;
        case 1:
            drawWeightChart(OLED,toyLift->currentLoad,toyLift->maxDynamicLoad);
            break;

        case 2:
            driverSelected=showDriver(OLED, &lastUpdate,driverNames,toyLift);
            break;

        default:
            drawBalanceChart(OLED,toyLift->loadBalance);
            break;
        }
        
        sendMQTT(MQTT,toyLift,RTLS);

        uint32_t mainLoopEndTime=esp_timer_get_time()/1000;
        uint16_t computeTime=mainLoopEndTime-mainLoopStartTime;
        //ESP_LOGI("main",":%d",RTLS->anchorId);
     


        //ESP_LOGI("main","left RAW: %ld KG: %f || right RAW:  %ld  KG: %f",leftForkSensor->Weight,leftForkSensor->kgWeight,rightForkSensor->Weight,rightForkSensor->kgWeight);
        //ESP_LOGI("main","noseDB: %f | HT: %d | CHT: %d",microphone->dbLevel,microphone->dbHigh,microphone->dbHighSustained);
        //ESP_LOGI("main","maxAccel: %f | currentAccel: %f",toyLift->maxAcceleration,toyLift->currentAccel);
        ESP_LOGI("main","pitch: %d | maxLoad: %f kg| currentLoad: %f kg| height: %dcm|GAS:%d|UID:%02X%02X%02X%02X|location:%d|ct:%d",IMU->pitch,toyLift->maxDynamicLoad,toyLift->currentLoad,TOF->distance,toyLift->gas,toyLift->currentDriver[0], toyLift->currentDriver[1], toyLift->currentDriver[2], toyLift->currentDriver[3],RTLS->anchorId,computeTime);
        //------------OUTPUTS AND NETWORKING-------------


        vTaskDelay(pdMS_TO_TICKS(40));
    }
}


