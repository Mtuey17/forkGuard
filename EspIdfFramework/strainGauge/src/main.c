#include "straingauge.h"
#include "driver/uart.h"
#include <string.h>
#include "esp_log.h"



#define RSTX 34
#define RSRX 35

void app_main() {


HX711_t *leftForkSensor=HX711_init(UART_NUM_1,RSRX,RSTX,115200,1);




while (true){

HX711_updateWeight(leftForkSensor,20);
ESP_LOGI("MAIN", "Current Force: %d", leftForkSensor->Weight);
vTaskDelay(pdMS_TO_TICKS(50));

}

}



/*
other UART functions 
char* test_str = "This is a test string.\n";
uart_write_bytes(RS485, (const char*)test_str, strlen(test_str));
vTaskDelay(pdMS_TO_TICKS(10));

// Write data to UART, end with a break signal.
//uart_write_bytes_with_break(uart_num, "test break\n",strlen("test break\n"), 100);

//read data
uint8_t data[128];
int length = 0;//get buffer size 
uart_get_buffered_data_len(RS485, (size_t*)&length);
length = uart_read_bytes(RS485, data, length, 100);
uart_flush(RS485);

//esp32 only supports built-in half duplex RS485
//uart_set_mode(RS485, UART_MODE_RS485_HALF_DUPLEX);
*/


