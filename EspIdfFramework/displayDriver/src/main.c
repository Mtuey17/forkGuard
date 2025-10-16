
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include <stdio.h>
#include <font_latin_8x8.h>
#include <ssd1306.h>
#include <bitmap_icon.h>
#include <bdf_font_emoticon_22x21.h>
#include <bdf_font_nenr12_21x26.h>



i2c_master_bus_handle_t i2c0_bus_hdl = NULL;

static const char *APP_TAG = "APP";

void i2c0_ssd1306_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ssd1306_config_t dev_cfg         = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ssd1306_handle_t dev_hdl;
    //
    // init device
    ssd1306_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ssd1306 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SSD1306 - START #########################");
        //
        int center = 1, top = 1;//bottom = 4;
     
        uint8_t image[24];

        ESP_LOGI(APP_TAG, "Panel is 128x64");

        // Display x3 text
        ESP_LOGI(APP_TAG, "Display x3 Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_contrast(dev_hdl, 0xff);
        ssd1306_display_text_x3(dev_hdl, 0, "Hello", false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Display bitmap icons
        ESP_LOGI(APP_TAG, "Display bitmap icons");
        ssd1306_set_contrast(dev_hdl, 0xff);
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 31, 0, data_rx_icon_32x32, 32, 32, false);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ssd1306_display_bitmap(dev_hdl, 31, 0, data_rx_icon_32x32, 32, 32, false);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Display x2 text
        ESP_LOGI(APP_TAG, "Display x2 Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_contrast(dev_hdl, 0xff);
        ssd1306_display_text_x2(dev_hdl, 0, "{xTEXTx}", false);
        ssd1306_display_text_x2(dev_hdl, 2, " X2-X2", false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Display text
        ESP_LOGI(APP_TAG, "Display Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, 0, "SSD1306 128x64", false);
        ssd1306_display_text(dev_hdl, 1, "Hello World!!", false);
        ssd1306_display_text(dev_hdl, 2, "SSD1306 128x64", true);
        ssd1306_display_text(dev_hdl, 3, "Hello World!!", true);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        
        // Display Count Down
        ESP_LOGI(APP_TAG, "Display Count Down");
        memset(image, 0, sizeof(image));
        ssd1306_display_image(dev_hdl, top, (6*8-1), image, sizeof(image));
        ssd1306_display_image(dev_hdl, top+1, (6*8-1), image, sizeof(image));
        ssd1306_display_image(dev_hdl, top+2, (6*8-1), image, sizeof(image));
        for(int font = 0x39; font > 0x30; font--) {
            memset(image, 0, sizeof(image));
            ssd1306_display_image(dev_hdl, top+1, (7*8-1), image, 8);
            memcpy(image, font_latin_8x8_tr[font], 8);
            if (dev_hdl->dev_config.flip_enabled) ssd1306_flip_buffer(image, 8);
            ssd1306_display_image(dev_hdl, top+1, (7*8-1), image, 8);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        

      



       
     

        // Bitmaps
        ESP_LOGI(APP_TAG, "Bitmaps");
        ssd1306_display_text(dev_hdl, 1, "BATMAN", false);
        int bitmapWidth = 4*8;
        int width = dev_hdl->width;
        int xpos = width / 2; // center of width
        xpos = xpos - bitmapWidth/2; 
        int ypos = 16;
        ESP_LOGD(APP_TAG, "width=%d xpos=%d", width, xpos);
        ssd1306_display_bitmap(dev_hdl, xpos, ypos, batman_icon_32x13, 32, 13, false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        for(int i=0;i<128;i++) {
            ssd1306_display_wrap_around(dev_hdl, SSD1306_SCROLL_RIGHT, 2, 3, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 0, 0, skull_icon_24x32, 24, 32, false);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i=0;i<64;i++) {
            ssd1306_display_wrap_around(dev_hdl, SSD1306_SCROLL_UP, 0, 127, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 0, 0, proton_icon_32x32, 32, 32, false);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Invert
        ESP_LOGI(APP_TAG, "Invert");
        ssd1306_clear_display(dev_hdl, true);
        ssd1306_set_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, center, "  Good Bye!!", true);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        // Fade Out
        ESP_LOGI(APP_TAG, "Fade Out");
        ssd1306_display_fadeout(dev_hdl);
        //
        ESP_LOGI(APP_TAG, "######################## SSD1306 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));

    }
    
    // free resources
    ssd1306_delete( dev_hdl );
    vTaskDelete( NULL );
}


void app_main() {

     // 1. Configure I²C
    i2c_master_bus_config_t i2c_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = 13,
    .sda_io_num = 12,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };

    // 2. Create I²C bus
 
    i2c_new_master_bus(&i2c_conf, &i2c0_bus_hdl);

    if (i2c0_bus_hdl == NULL) {
        ESP_LOGE(APP_TAG, "I2C bus init failed");
        return;
    }
    // 3. Start SSD1306 demo task
    xTaskCreate(i2c0_ssd1306_task, "ssd1306_task", 4096, NULL, 5, NULL);

}