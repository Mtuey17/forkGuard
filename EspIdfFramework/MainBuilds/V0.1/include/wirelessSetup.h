#include "mqtt_client.h"
#include "liftDynamics.h"
#ifndef WIRELESSSETUP_H
#define WIRELESSSETUP_H


typedef struct {
    
    bool weightLatch;
    bool brakeLatch;
    bool gasLatch;
    bool noiseLatch;
    bool UUIDLAtch;
    esp_mqtt_client_handle_t handler;
   
} MQTTHandler;
void initWifi(char*,char*);
MQTTHandler* initMQTT();
bool sendMQTT(MQTTHandler*,ForkLift*);




#endif 
