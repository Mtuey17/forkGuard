/* Matthew Tuer, July 30, 2025 mtuer@uwaterloo.ca/matthewjtuer@gmail.com */

#ifndef STRAINGAUGE_H
#define STRAINGAUGE_H

#include <stdint.h>
#include "driver/uart.h"  

typedef struct {
    uart_port_t uart_num;     
    uint8_t deviceID;         
    long Weight;               
    uint8_t requestCRC[2];   
    int32_t offset;           
    bool offsetCheck;      
} HX711_t;

HX711_t *HX711_init(uart_port_t uart_num, uint8_t Rx,uint8_t Tx, uint32_t baudRate,uint8_t id);

long HX711_updateWeight(HX711_t *hx, uint16_t timeout_ms);

uint16_t HX711_crc16(uint8_t *data, uint8_t len);

#endif 
