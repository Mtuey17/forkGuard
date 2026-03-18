#include "wirelessSetup.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "liftDynamics.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "RTLS.h"
#include "esp_timer.h"
#include <math.h>
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
MQTT_INSTANCE->weightErrorLatch=false;
MQTT_INSTANCE->brakeErrorLatch=false;
MQTT_INSTANCE->handler=client;
return MQTT_INSTANCE;

}

bool sendMQTT(MQTTHandler* MQTT_INSTANCE,ForkLift* LiftValues,RTLS_Instance* RT){
    float safeMargine=LiftValues->safetyFactor;


    
    //--------------Handling brake warning and error messages--------------
    //approaching max 
    bool allowWeightWarning=true;



    if (LiftValues->currentAccel<-4.5f&&!MQTT_INSTANCE->brakeLatch){
        char msg[128];
        snprintf(msg, sizeof(msg), "hard brake!");
        gpio_set_level(GPIO_NUM_45,1);
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/brakeWarning", msg, 0, 2, 0);
        MQTT_INSTANCE->brakeLatch=true;
    }
    else{}
    if (LiftValues->currentAccel>-4.4f){ gpio_set_level(GPIO_NUM_45,0); MQTT_INSTANCE->brakeLatch=false;}



    //--------------Handling weight warning and error messages--------------
    //when accelerating/deaccelerating hard weight reading may be incorrect. only allowed to send weight warning when not under steep accel
    //case when driver is operating within safety margine
    //apporaching max


    if (fabs(LiftValues->currentLoad-2.6f)>150.0f){
    if (((LiftValues->currentLoad/LiftValues->maxDynamicLoad)>safeMargine)&&!MQTT_INSTANCE->weightLatch&&allowWeightWarning){
    MQTT_INSTANCE->weightLatch=true;
    char msg[128];

        gpio_set_level(GPIO_NUM_48,1);
        snprintf(msg, sizeof(msg), "Approaching_Weight_Max");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/weightWarning", msg, 0, 2, 0);

    }
    if ((LiftValues->currentLoad/LiftValues->maxDynamicLoad)<safeMargine){
    MQTT_INSTANCE->weightLatch=false;
    gpio_set_level(GPIO_NUM_48,0);
    }
    //exceeding max 
    if (LiftValues->currentLoad>=LiftValues->maxDynamicLoad){
    if (LiftValues->currentLoad>=LiftValues->maxDynamicLoad&&!MQTT_INSTANCE->weightErrorLatch){
        MQTT_INSTANCE->weightErrorLatch=true;
        char msg[128];

        snprintf(msg, sizeof(msg), "Excedded_Weight_Max");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/weightError", msg, 0, 2, 0);
    }}
     if (LiftValues->currentLoad<LiftValues->maxDynamicLoad){
        MQTT_INSTANCE->weightErrorLatch=false;
     }
    }
     //-----------------------------------------------------------------------



    //----------------------gas message-----------------------------------
    if (!LiftValues->gas&& !MQTT_INSTANCE->gasLatch){
        MQTT_INSTANCE->gasLatch=true;
        char msg[128];
        snprintf(msg, sizeof(msg), "Gas_Level_High");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/gas", msg, 0, 2, 0);
    }
    if (LiftValues->gas){
        MQTT_INSTANCE->gasLatch=false;
    }
    //--------------------------------------------------------------------

    //----------------------noise message-----------------------------------
    if (LiftValues->noise&& !MQTT_INSTANCE->noiseLatch){
        MQTT_INSTANCE->noiseLatch=true;
        char msg[128];
        snprintf(msg, sizeof(msg), "Noise_Level_High");
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/noise", msg, 0, 2, 0);
    }
    if (!LiftValues->noise){
        MQTT_INSTANCE->noiseLatch=false;
    }
    //--------------------------------------------------------------------


  
    
    //-------------------UUID message-------------------------------------
    if (memcmp(LiftValues->currentDriver,LiftValues->previousDriver,sizeof(LiftValues->currentDriver)) != 0){

        memcpy(LiftValues->previousDriver,LiftValues->currentDriver,sizeof(LiftValues->currentDriver));
        char msg[128];
        snprintf(msg, sizeof(msg), "%02X%02X%02X%02X",LiftValues->currentDriver[0],LiftValues->currentDriver[1],LiftValues->currentDriver[2],LiftValues->currentDriver[3]);
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/driver", msg, 0, 2, 0);
    }
    //----------------------------------------------------------------
    
    uint16_t rateMs=(1/RT->sendRateHz)*1000;
    uint32_t currentTime=esp_timer_get_time()/1000;
    if ((currentTime-RT->lastSendTime)>=rateMs)
    {
        RT->lastSendTime=currentTime;
        char msg[128];
        snprintf(msg, sizeof(msg), "%d",RT->anchorId);
        RT->previousId=RT->anchorId;
        esp_mqtt_client_publish(MQTT_INSTANCE->handler, "toyLift1/Location", msg, 0, 2, 0);
    }
    


return true;
}