#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_io_i2c.h"

#define INPUT_PIN GPIO_NUM_4   // Example: GPIO4
void app_main() {

    gpio_config_t io_conf = {
        //.intr_type = GPIO_INTR_DISABLE,   // No interrupts
        .mode = GPIO_MODE_INPUT,          // Set as input
        .pin_bit_mask = (1ULL << INPUT_PIN), // Bit mask for the pin
        //.pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .pull_up_en = GPIO_PULLUP_ENABLE,     // Enable pull-up (optional)
    };
    gpio_config(&io_conf);

    while (1) {
        int level = gpio_get_level(INPUT_PIN);  // Read pin state
        printf("GPIO %d level: %d\n", INPUT_PIN, level);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1s
    }


}