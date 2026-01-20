#include "driver/i2c_master.h"
#include "VL53L.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SDAPIN 14
#define SCLPIN 12

void app_main() {

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



    //page 60 of datasheet for id, maybe 28 0r 29
    VL53L_t *TOF=initializeTOF(bus_handle,0x29);


    while(1){

        updateDistance(TOF);
        ESP_LOGI("TOF", "id: %d", TOF->validID);
        ESP_LOGI("TOF", "Distance: %d cm", TOF->distance);
        vTaskDelay(pdMS_TO_TICKS(100));


        
 
        
        
    }


}