/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
c file for BNO055 IMU
*/



#include "OLED.h"
#include "driver/i2c_master.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h"
#include "liftDynamics.h"
#include <string.h>



ssd1306_handle_t initializeOLED(i2c_master_bus_handle_t MasterHandle){

    // initialize the xLastWakeTime variable with the current time.
    //TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ssd1306_config_t dev_cfg         = I2C_SSD1306_128x32_CONFIG_DEFAULT;
    ssd1306_handle_t dev_hdl;
    //
    // init device
    ssd1306_init(MasterHandle, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE("OLED", "ssd1306 handle init failed");
        return NULL;
    }

return dev_hdl;
}



/*
OLD method, too slow to draw 
void drawWeightChart(ssd1306_handle_t OLED,float currentWeight,float maxCurrentWeight){


    //ssd1306_clear_display(OLED, false);


    float matFlatWeight=0.49;
    float maxWeigthIndex=120*(fminf((maxCurrentWeight/matFlatWeight),1.00));
    float safetyScoreThreshold=maxWeigthIndex*0.75;//user should only go up to 75% of technical max 
    float weightPrecentage=(currentWeight/maxCurrentWeight)*100;
    
    ssd1306_set
    ssd1306_display_bitmap(OLED, (int)maxWeigthIndex, 0, whiteLine_icon_32x32, 1, 32, false);
    ssd1306_display_bitmap(OLED, (int)safetyScoreThreshold, 0, whiteLine_icon_32x32, 1, 32, false);


    float xIndex=maxWeigthIndex*(weightPrecentage/100);
    float heightCount=0;
    for (int i=0;i<(int)xIndex;i++){
        heightCount++;
        float mapHeight=32*(heightCount/maxWeigthIndex);
        ssd1306_display_bitmap(OLED,i, 32-(int)mapHeight, whiteLine_icon_32x32, 1, (int)mapHeight, false);
       
    }


}
    */

   void drawWeightChart(ssd1306_handle_t OLED,float currentWeight,float maxCurrentWeight)
{
    const float matFlatWeight = 0.49f;
    const uint8_t maxHeight = 32;

    float maxWidth = 120.0f * fminf(maxCurrentWeight / matFlatWeight, 1.0f);
    float safeX    = maxWidth * 0.80f;

    float pct = currentWeight / maxCurrentWeight;
    pct = fminf(pct, 1.0f);

    uint8_t rampWidth = (uint8_t)(maxWidth * pct);


    memset(OLED->page, 0, sizeof(OLED->page));

    
    ssd1306_set_line(OLED, (int)maxWidth, 0, (int)maxWidth, 31, false);
    ssd1306_set_line(OLED, (int)safeX,   0, (int)safeX,   31, false);


    for (uint8_t x = 0; x < rampWidth; x++)
    {
    
        float h = ((float)x / maxWidth) * maxHeight;

        uint8_t height = (uint8_t)(h * pct);
        if (height == 0) continue;

        uint8_t yTop = maxHeight - height;


        ssd1306_set_line(OLED, x, yTop, x, maxHeight - 1, false);
    }

    ssd1306_display_pages(OLED);
}


void drawBalanceChart(ssd1306_handle_t OLED, float imbalancePercent)
{
    const int squareSize = 16;
    const float precentage=100.0f;

    const int centerX = 128 / 2;
    const int centerY = 32 / 2;

    const int maxOffset = (128 / 2) - (squareSize / 2) - 2;
    const int leftLimit  = centerX - maxOffset;
    const int rightLimit = centerX + maxOffset;

    if (imbalancePercent > precentage) imbalancePercent = precentage;
    if (imbalancePercent < -precentage) imbalancePercent = -precentage;

    float norm = -imbalancePercent / precentage;

    int squareX = centerX - (squareSize / 2) + (int)(norm * maxOffset);
    int squareY = centerY - (squareSize / 2);

    memset(OLED->page, 0, sizeof(OLED->page));

    ssd1306_set_line(OLED, centerX, 0, centerX, 32 - 1, false);
    ssd1306_set_line(OLED, leftLimit, centerY - 8, leftLimit, centerY + 8, false);
    ssd1306_set_line(OLED, rightLimit, centerY - 8, rightLimit, centerY + 8, false);

    for (int x = squareX; x < squareX + squareSize; x++)
    {
        ssd1306_set_line(OLED, x, squareY, x, squareY + squareSize - 1, false);
    }

    ssd1306_display_pages(OLED);
}

bool showDriver(ssd1306_handle_t OLED,uint32_t *lastUpdate,const char*driverNames[],ForkLift*FL)
{
    
    
    const float refreshRateHz=5.0;
    bool driverSelected=true;
    const char *currentDriver=driverNames[FL->currentDriver[0]];

    if (currentDriver!=NULL){
    if (strcmp(currentDriver, "None") == 0) {driverSelected = false;}
    };
    
    uint32_t periodMs=(1.0f/refreshRateHz)*1000.0f;
    uint32_t currentTime=esp_timer_get_time()/1000;
    if ((currentTime-*lastUpdate)>=periodMs){
        *lastUpdate=currentTime;  
        ssd1306_display_text_x2(OLED,0,"Driver:",false);
        if (currentDriver==NULL){currentDriver="Unknown";}
        char toDisplay[128];
        snprintf(toDisplay, sizeof(toDisplay), "%s",currentDriver);
        ssd1306_display_text_x2(OLED,2,toDisplay,false);
        
    }
        
    return driverSelected;




}
void showWeightInfo(ssd1306_handle_t OLED, uint32_t *lastUpdate,ForkLift* FL){
    const float refreshRateHz=5.0;
    uint32_t periodMs=(1.0f/refreshRateHz)*1000.0f;
    uint32_t currentTime=esp_timer_get_time()/1000;
    if ((currentTime-*lastUpdate)>=periodMs){//only refreshing screen at rate of 'refreshRateHz'
        *lastUpdate=currentTime;
        char toDisplay[128];

        //ssd1306_clear_display(OLED,false);

        //ssd1306_display_text_x2(OLED,0,"Load",false);


        snprintf(toDisplay, sizeof(toDisplay), "ML:%0.3f",FL->maxDynamicLoad);
        ssd1306_display_text_x2(OLED,0,toDisplay,false);
        snprintf(toDisplay, sizeof(toDisplay), "CL:%0.3f",FL->currentLoad);
        ssd1306_display_text_x2(OLED,2,toDisplay,false);


        /*
        snprintf(toDisplay, sizeof(toDisplay), "Driver: %s",currentDriver);
        ssd1306_display_text(OLED,0,toDisplay,false);
        snprintf(toDisplay, sizeof(toDisplay), "Max Load: %0.3f",FL->maxDynamicLoad);
        ssd1306_display_text(OLED,1,toDisplay,false);
        snprintf(toDisplay, sizeof(toDisplay), "C-Load: %0.3f",FL->currentLoad);
        ssd1306_display_text(OLED,2,toDisplay,false);
        snprintf(toDisplay, sizeof(toDisplay), "M-Brake: %0.1fm/s",FL->maxAcceleration);
        ssd1306_display_text(OLED,3,toDisplay,false);
        snprintf(toDisplay, sizeof(toDisplay), "M-Brake: %0.1fm/s",FL->maxAcceleration);
        ssd1306_display_text(OLED,4,toDisplay,false);
        */



    }





}

const uint8_t forklift_guard_icon_64x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0x08, 0x08, 0x08,
    0x88, 0x88, 0x88, 0x08, 0x08, 0x08, 0x18, 0x30, 0xE0, 0x80, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x80,
    0x00, 0x00, 0x00, 0x70, 0xFE, 0xFF, 0xF8, 0xF8, 0xF8, 0xF8, 0xFC, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF,
    0xFE, 0xFE, 0xFC, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x03, 0xE0, 0xC0, 0xF0,
    0xFB, 0xFB, 0xF3, 0x60, 0x60, 0x60, 0x20, 0x70, 0xC1, 0x8F, 0xFC, 0xE0, 0x00, 0x0F, 0xFF, 0xFF,
    0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF0, 0x80, 0xC1, 0xE7, 0xFF, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF,
    0x1F, 0x00, 0x00, 0x1C, 0x3F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F,
    0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x00, 0x00, 0xF0, 0xF8, 0xFC, 0xFF, 0x07, 0x07, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x83, 0xE7, 0xFF, 0x3F, 0x1E, 0x0F, 0x03, 0x01, 0x01, 0x07, 0x1F, 0x7F, 0xFE, 0xF8,
    0xF8, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07, 0xFF, 0xFE, 0xF8, 0xE0, 0x00, 0x00, 0x00
};


/*
****************OLED FUNCTIONS******************

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
    .scl_io_num = 18,
    .sda_io_num = 19,
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
    
    ssd1306_handle_t OLED=initializeOLED(i2c0_bus_hdl);
    ssd1306_display_bitmap(OLED, 0, 0, skull_icon_24x32, 24, 32, false);

*/





