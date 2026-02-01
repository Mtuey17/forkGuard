/* Matthew Tuer 
november 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
*/
#ifndef ICS43434_H
#define ICS43434_H
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"



typedef struct {

    float warningSustainedDB;
    float warningInstantDB;
    float sustainedSeconds;
    bool FPS;
    uint16_t sampleRate;
    uint16_t frameSamples;
    float offset;
    i2s_chan_handle_t handle; 
    int32_t buffer[2048];

    int frameOverCount;
    float dbLevel;
    bool dbHigh;
    bool dbHighSustained;
     
}  ICS43434;

ICS43434* InitICS(i2s_chan_handle_t);

bool checkMicLevel(ICS43434*);

#endif 
