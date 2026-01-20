
/* Matthew Tuer 
october 14th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 

source code for sprint 1 demonstration

this code demonstrates:

ESP32 connecting to a wireless network
ESP32 connecting to a MQTT server VIA a wireless network
ESP32 sending simulated sensor values over MQTT server, where it is stored in a database

this code also includes:

driver for load sensor 
driver for IMU 
Custom made functions for Wireless and MQTT setup

*/


#include "driver/uart.h"
#include "driver/i2c_master.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "mqtt_client.h"
#include "esp_random.h"

//custom made files 
#include "wirelessSetup.h"
#include "straingauge.h"
#include "IMU.h"


//RS485 Pins
#define RSTX 32
#define RSRX 35

//i2c pins 
#define SDAPIN 15
#define SCLPIN 16

#define SWITCH_PIN GPIO_NUM_15
#define LED_PIN GPIO_NUM_22
#define BUZZER_PIN GPIO_NUM_23

#define WIFI_SSID "ForkGuardNet"
#define WIFI_PASS "Guard1234"

uint32_t rand_range(uint32_t min, uint32_t max) {
    return min + (esp_random() % (max - min + 1));
}

void app_main() {


    // --- GPIO setup ---
    gpio_set_direction(SWITCH_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(SWITCH_PIN);  

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);


    //wireless setup
    initWifi(WIFI_SSID,WIFI_PASS);
    esp_mqtt_client_handle_t mqttClient=initMQTT();


    //not used, waiting for part. using fake values for demo
    /*
    i2c_master_bus_config_t I2C_0_CONFIG = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCLPIN,
    .sda_io_num = SDAPIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&I2C_0_CONFIG, &bus_handle);
    IMU_t *IMU=initializeIMU(bus_handle,0x28);*/



    //not used, waiting for part. using fake value for demo
    //HX711_t *leftForkSensor=HX711_init(UART_NUM_1,RSRX,RSTX,115200,1);



while (true){


    //test switch-showcasing live user input data 
    int switchLevel = gpio_get_level(SWITCH_PIN);  // 1 = released, 0 = pressed
    int outputLevel=0;
    if (switchLevel==0){outputLevel=1;}

    gpio_set_level(LED_PIN,outputLevel);
    gpio_set_level(BUZZER_PIN,outputLevel);



    //fake random values
    int weight= rand_range(0,100);
    int yaw=rand_range(-25,25);
    int pitch=rand_range(-25,25);
    int roll=rand_range(-25,25);
    int accel=rand_range(-10,10);


    //data upload
    char MQTTbuffer[100];
    snprintf(MQTTbuffer, sizeof(MQTTbuffer), "weight:%d,yaw:%d,pitch:%d,roll:%d,accel:%d,switch:%d",weight, yaw, pitch, roll, accel,switchLevel);
    esp_mqtt_client_publish(mqttClient, "sensorValues", MQTTbuffer, 0, 0, 0);


    //prints
    ESP_LOGI("MAIN", "%s", MQTTbuffer);  

    vTaskDelay(pdMS_TO_TICKS(200));


}

}