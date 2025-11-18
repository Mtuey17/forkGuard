
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"


#include "OLED.h"
#include <stdio.h>
#include <font_latin_8x8.h>
#include <ssd1306.h>
#include <bitmap_icon.h>
#include <bdf_font_emoticon_22x21.h>
#include <bdf_font_nenr12_21x26.h>



i2c_master_bus_handle_t i2c0_bus_hdl = NULL;





void app_main() {

     // 1. Configure I²C
    i2c_master_bus_config_t i2c_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = 18,
    .sda_io_num = 19,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
    // 2. Create I²C bus
    i2c_new_master_bus(&i2c_conf, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) {
        ESP_LOGE("I2C", "I2C bus init failed");
        return;
    }

    
    ssd1306_handle_t OLED=initializeOLED(i2c0_bus_hdl);//initialize OLED

    drawWeightChart(OLED,75.1);
    
   
 

  
}