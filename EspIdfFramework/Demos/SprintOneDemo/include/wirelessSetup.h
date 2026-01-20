/* Matthew Tuer 
october 3rd, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
Header file for wireless communication functions
*/

#include "mqtt_client.h"

#ifndef WIRELESSSETUP_H
#define WIRELESSSETUP_H

void initWifi(char*,char*);
esp_mqtt_client_handle_t initMQTT();


#endif 
