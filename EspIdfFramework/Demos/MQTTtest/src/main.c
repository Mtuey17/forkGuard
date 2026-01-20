#include "driver/gpio.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <string.h>
#include "mqtt_client.h"
#include "wirelessSetup.h"

#define SWITCH_PIN GPIO_NUM_15
#define BUZZER_PIN GPIO_NUM_33
#define LED_PIN GPIO_NUM_32


#define WIFI_SSID "ForkGuardNet"
#define WIFI_PASS "Guard1234"

void app_main(void) {
    // --- GPIO setup ---
    gpio_set_direction(SWITCH_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(SWITCH_PIN);  

    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  
    initWifi(WIFI_SSID,WIFI_PASS);
    esp_mqtt_client_handle_t mqttClient=initMQTT();

    // --- Main loop ---
    while (1) {
        int switchLevel = gpio_get_level(SWITCH_PIN);  // 1 = released, 0 = pressed
        ESP_LOGI("SWITCH", "Level = %d", switchLevel);
        gpio_set_level(LED_PIN,switchLevel);
        gpio_set_level(BUZZER_PIN,switchLevel);
        vTaskDelay(pdMS_TO_TICKS(100));  
   
        char msg[16];
        snprintf(msg, sizeof(msg), "%d", switchLevel);
        esp_mqtt_client_publish(mqttClient, "test", msg, 0, 1, 0);
        
        

    }
}


