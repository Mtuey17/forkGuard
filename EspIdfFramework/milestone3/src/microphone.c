// CZN-15E ESP32
// Loud Noise Detector (module)
// Created: 2025-10-28
// Jackson Whynott

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "microphone.h"

static const char *TAG = "Loud_Noise";

void microphoneInit(void)
{






    gpio_set_direction(D0_PIN, GPIO_MODE_INPUT);

    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(RED_LED, 0);






}

int microphoneLevel()
{
    
        // triggered will go true when a loud noise is made
        int triggered = gpio_get_level(D0_PIN);
        if (triggered == 1)
        {gpio_set_level(RED_LED, 1);}
        else{gpio_set_level(RED_LED, 0);}
        return triggered;

        
}
