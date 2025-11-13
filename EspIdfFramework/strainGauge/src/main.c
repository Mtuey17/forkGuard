#include "straingauge.h"
#include "driver/uart.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#define RSTX 23
#define RSRX 22

void app_main() {


HX711_t *leftForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,1);
HX711_t *rightForkSensor=HX711_init(UART_NUM_2,RSRX,RSTX,115200,2);
bool flipStrainRead=false;
while (true){
int64_t start_time = esp_timer_get_time(); 


    if (flipStrainRead){HX711_updateWeight(leftForkSensor, 20);}
    else{HX711_updateWeight(rightForkSensor, 20);}
    flipStrainRead=!flipStrainRead;
    
    int64_t end_time = esp_timer_get_time();   
    int64_t compute_time_us = end_time - start_time;
    float compute_time_ms = compute_time_us / 1000.0; // Convert to milliseconds
    ESP_LOGI("MAIN", "Left Force: %d||Right Force: %d||Compute Time: %f", leftForkSensor->Weight,rightForkSensor->Weight,compute_time_ms);
    vTaskDelay(pdMS_TO_TICKS(30));
}

}




