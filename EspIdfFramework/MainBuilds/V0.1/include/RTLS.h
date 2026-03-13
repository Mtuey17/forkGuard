/* Matthew Tuer 
october 15th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
*/

#ifndef RTLS_H
#define RTLS_H

#include <stdint.h>
#include "driver/uart.h"  

typedef struct {
    uart_port_t uart_num; 
    char location [8];    
    int anchorId;
    int previousId;
    float sendRateHz;
    uint32_t lastSendTime;
        

} RTLS_Instance;

RTLS_Instance *RTLS_init(uart_port_t uart_num, uint8_t Rx,uint8_t Tx, uint32_t baudRate);

bool UPDATE_LOCATION(RTLS_Instance *RT_Instance, uint16_t timeout_ms);


#endif 
