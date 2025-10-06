#include "straingauge.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <stdio.h>



HX711_t *HX711_init(uart_port_t uart_num, uint8_t id){

    HX711_t *hx_instance = malloc(sizeof(HX711_t));
    hx_instance->uart_num = uart_num;
    hx_instance->deviceID = id;
    hx_instance->Weight = 0;
    hx_instance->requestCRC[0] = 0;
    hx_instance->requestCRC[1] = 0;
    hx_instance->offset = 0;
    hx_instance->offsetCheck = false;
    return hx_instance;

}

long HX711_updateWeight(HX711_t *hx, uint16_t timeout_ms){

    uart_flush(hx->uart_num);

    uint8_t toSend[]={ hx->deviceID, 0x03, 0x00, 0x00, 0x00, 0x03 };
    int crc=HX711_crc16(toSend,6);
    int toSendCRC[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
    uart_write_bytes(hx->uart_num, (const char*)toSend,6);
    uart_write_bytes(hx->uart_num, (const char*)toSendCRC,2);
    vTaskDelay(pdMS_TO_TICKS(5));

    int startTime=esp_timer_get_time()/1000;
    int incomingDataLength=0;
    while (incomingDataLength<9){
        uart_get_buffered_data_len(hx->uart_num, (size_t*)&incomingDataLength);
        int currentTime=esp_timer_get_time()/1000;
        if (currentTime-startTime>timeout_ms){
            return hx->Weight;
        }
    }
    int deviceResponse[11];
    int bytesToRead=11;
    uart_read_bytes(hx->uart_num, deviceResponse, bytesToRead,portMAX_DELAY);
    
    int data=((int32_t)deviceResponse[3] << 24) |
                ((int32_t)deviceResponse[4] << 16) |
                ((int32_t)deviceResponse[5] << 8)  |
                (int32_t)deviceResponse[6];
    if (!hx->offsetCheck){
        hx->offset=data;
        hx->offsetCheck=true;
        hx->Weight=0;
        return hx->Weight;
    }
    data-=hx->offset;
    hx->Weight=data;
    return hx->Weight;
}

uint16_t HX711_crc16(uint8_t *data, uint8_t len){

    int crc=0xFFFF;
    for (int i=0;i<len;i++){
        crc^=data[i];
        for (int j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
    }
     return crc;
}