#include "microphone.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"
#include <math.h>
#include "esp_log.h"
ICS43434* InitICS(i2s_chan_handle_t i2s_handler){


    ICS43434 *ICS_Instance = malloc(sizeof(ICS43434));
    ICS_Instance->handle=i2s_handler;
    ICS_Instance->frameSamples=1024;
    ICS_Instance-> warningSustainedDB=85.0f;
    ICS_Instance-> warningInstantDB=100.0f;
    ICS_Instance-> sustainedSeconds=3.0f; 
    ICS_Instance-> offset=123.0103f;
    //ICS_Instance-> buffer[ICS_Instance->frameSamples*2];
    ICS_Instance-> frameOverCount=0;
    ICS_Instance-> dbLevel=0;
    ICS_Instance-> dbHigh=false;
    ICS_Instance-> dbHighSustained=false;
    ICS_Instance->FPS=48000/1024;
    return ICS_Instance;
}

bool checkMicLevel(ICS43434* ICS_Instance){
size_t bytesRead = 0;
esp_err_t err = i2s_channel_read(ICS_Instance->handle, ICS_Instance->buffer,
                                sizeof(ICS_Instance->buffer),
                                &bytesRead, pdMS_TO_TICKS(50));

if (bytesRead == 0) return false;

int n = bytesRead / sizeof(int32_t);
int32_t mn = INT32_MAX, mx = INT32_MIN;
for (int i = 0; i < n; i++) {
    int32_t v = ICS_Instance->buffer[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
}






    double sumSquare = 0.0;
    int count = 0;

    // Mic sends data left, right, left, right. This reads only lefts since L/R is strapped to ground
    for (int i = 0; i + 1 < n; i += 2) {
        // Common alignment: 24-bit data in top bits of 32-bit slot
        int32_t s24 = ICS_Instance-> buffer[i] >> 8;
        float normalized = (float)s24 / (float)(1 << 23);  // normalizes 24 bits to ~-[1,1]
        sumSquare += (double)normalized * (double)normalized;
        count++; // number of samples in frame
    }
    if (count==0){return false;}

    float meanSquare = (float)(sumSquare / (double)count);
    float rms = sqrtf(fmaxf(meanSquare, 1e-20f)); 
    float dbfs_rms = 20.0f * log10f(rms);
    ICS_Instance->dbLevel=dbfs_rms + ICS_Instance-> offset;

    if (ICS_Instance->dbLevel>=ICS_Instance->warningInstantDB){ ICS_Instance->dbHigh=true;}
    else{ICS_Instance->dbHigh=false;}
    if (ICS_Instance->dbLevel>=ICS_Instance->warningSustainedDB){
        ICS_Instance->frameOverCount++;
        if (ICS_Instance->frameOverCount>=(ICS_Instance->FPS*ICS_Instance->sustainedSeconds)){
            ICS_Instance->dbHighSustained=true;
        }
    }
    else{
      ICS_Instance->frameOverCount=0;  
      ICS_Instance->dbHighSustained=false;

    }
    

    return true; 
}