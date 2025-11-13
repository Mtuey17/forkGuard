#include "straingauge.h"
#include "driver/uart.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "IMU.h"
#include <stdio.h>


#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "wirelessSetup.h"


#define RSTX 23
#define RSRX 22

#define SDAPIN 14
#define SCLPIN 12

#define WIFI_SSID "ForkGuardNet"
#define WIFI_PASS "Guard1234"

void app_main() {


    //i2c setup
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


    //imu setup
    IMU_t *IMU=initializeIMU(bus_handle,0x29);



    //strain amp init
    //HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1);
    //HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2);
    bool flipStrainRead=false;

    //wifi/mqtt init 
    //initWifi(WIFI_SSID,WIFI_PASS);
    //esp_mqtt_client_handle_t mqttClient=initMQTT();

while (true){
int64_t start_time = esp_timer_get_time(); 


    //if (flipStrainRead){HX711_updateWeight(leftForkSensor, 20);}
    //else{HX711_updateWeight(rightForkSensor, 20);}
    flipStrainRead=!flipStrainRead;
    




    updateAngles(IMU);
    updateAcceleration(IMU);

    int64_t end_time = esp_timer_get_time();   
    int64_t compute_time_us = end_time - start_time;
    float compute_time_ms = compute_time_us / 1000.0; // Convert to milliseconds
    ESP_LOGI("main","roll:%d|pitch:%d|accel:%f||Compute Time: %f",IMU->roll,IMU->pitch,IMU->forwardAcceleration,compute_time_ms);

    //ESP_LOGI("main","roll:%d|pitch:%d|accel:%f|Left Force: %d||Right Force: %d||Compute Time: %f",IMU->roll,IMU->pitch,IMU->forwardAcceleration,leftForkSensor->Weight,rightForkSensor->Weight,compute_time_ms);


    //writing data to mqtt 
    //char msg[50];
    //snprintf(msg, sizeof(msg), "r:%d,p:%d,a:%f,lf:%d,rf:%d,",IMU->roll,IMU->pitch,IMU->forwardAcceleration,leftForkSensor->Weight,rightForkSensor->Weight);
    //esp_mqtt_client_publish(mqttClient, "test", msg, 0, 1, 0);
    vTaskDelay(pdMS_TO_TICKS(30));

}

}




