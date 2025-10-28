#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GAS_PIN GPIO_NUM_23


void app_main() {


    gpio_set_direction(GAS_PIN,GPIO_MODE_INPUT);
    int gasDetected=0;
    while (1){

        gasDetected=gpio_get_level(GAS_PIN);
        ESP_LOGI("main", "Gas:%d",gasDetected);
        vTaskDelay(pdMS_TO_TICKS(100));


    }
}