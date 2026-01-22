#include "wirelessSetup.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "liftDynamics.h"
#include "driver/gpio.h"
#include "esp_log.h"
void initWifi(char *SSID,char *Password){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // must be called before creating default Wi-Fi STA
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, SSID);
    strcpy((char *)wifi_config.sta.password, Password);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}




MQTTHandler* initMQTT(){

     const esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri= "mqtt://192.168.8.210:1883",
};
esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
esp_mqtt_client_start(client);

MQTTHandler *MQTT_INSTANCE = malloc(sizeof(*MQTT_INSTANCE));
MQTT_INSTANCE->brakeLatch=false;
MQTT_INSTANCE->gasLatch=false;
MQTT_INSTANCE->noiseLatch=false;
MQTT_INSTANCE->weightLatch=false;
MQTT_INSTANCE->UUIDLAtch=false;
MQTT_INSTANCE->handler=client;
return MQTT_INSTANCE;

}

bool sendMQTT(MQTTHandler* MQTT_INSTANCE,ForkLift* LiftValues){
    float safeMargine=LiftValues->safetyFactor;

    //case when driver is operating within safety margine 
    if (((LiftValues->currentLoad/LiftValues->maxDynamicLoad)>safeMargine)&&!MQTT_INSTANCE->weightLatch){
    MQTT_INSTANCE->weightLatch=true;
    char msg[128];

        gpio_set_level(GPIO_NUM_48,1);
        snprintf(msg, sizeof(msg), "test");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/weightWarning", msg, 0, 2, 0);

    }
    if ((LiftValues->currentLoad/LiftValues->maxDynamicLoad)<safeMargine){
    MQTT_INSTANCE->weightLatch=false;
    gpio_set_level(GPIO_NUM_48,0);
    }

    //case when driver is operating within safety margine 
    if (((LiftValues->currentAccel/LiftValues->maxAcceleration)>safeMargine)&&!MQTT_INSTANCE->brakeLatch){
    MQTT_INSTANCE->brakeLatch=true;
    char msg[128];

        snprintf(msg, sizeof(msg), "tes2t");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/brakeWarning", msg, 0, 2, 0);

    }
    if ((LiftValues->currentAccel/LiftValues->maxAcceleration)<safeMargine){
    MQTT_INSTANCE->brakeLatch=false;
    }

    if (!LiftValues->gas&& !MQTT_INSTANCE->gasLatch){
        MQTT_INSTANCE->gasLatch=true;
        char msg[128];
        snprintf(msg, sizeof(msg), "test3");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/gas", msg, 0, 2, 0);
    }
    if (LiftValues->gas){
        MQTT_INSTANCE->gasLatch=false;
    }

    if (memcmp(LiftValues->currentDriver,LiftValues->previousDriver,sizeof(LiftValues->currentDriver)) != 0){

        memcpy(LiftValues->previousDriver,LiftValues->currentDriver,sizeof(LiftValues->currentDriver));
        char msg[128];
        snprintf(msg, sizeof(msg), "%02X%02X%02X%02X",LiftValues->currentDriver[0],LiftValues->currentDriver[1],LiftValues->currentDriver[2],LiftValues->currentDriver[3]);
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/driver", msg, 0, 2, 0);
    }
    
return true;
}