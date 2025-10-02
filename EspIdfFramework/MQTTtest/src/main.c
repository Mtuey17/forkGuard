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

#define SWITCH_PIN GPIO_NUM_14
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



    /*
    // --- NVS init (required for Wi-Fi) ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- TCP/IP stack and Wi-Fi ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // must be called before creating default Wi-Fi STA
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());


    const esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri= "mqtt://192.168.8.210:1883",
};
esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
esp_mqtt_client_start(client);*/



    // --- Main loop ---
    while (1) {
        int switchLevel = gpio_get_level(SWITCH_PIN);  // 1 = released, 0 = pressed
        ESP_LOGI("SWITCH", "Level = %d", switchLevel);
        gpio_set_level(LED_PIN,switchLevel);
        gpio_set_level(BUZZER_PIN,switchLevel);
        vTaskDelay(pdMS_TO_TICKS(100));  




        //char msg[16];
        //snprintf(msg, sizeof(msg), "%d", switchLevel);
        //esp_mqtt_client_publish(client, "test", msg, 0, 1, 0);
        

    }
}
