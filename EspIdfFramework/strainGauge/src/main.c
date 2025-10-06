#include "straingauge.h"
#include "driver/uart.h"
#include <string.h>
#include "esp_log.h"



#define RSTX 14
#define RSRX 15

void app_main() {

const uart_port_t RS485 = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
};
// Configure UART parameters
uart_param_config(RS485, &uart_config);
uart_set_pin(RS485, RSTX, RSRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
HX711_t *leftForkSensor=HX711_init(RS485,1);




while (true){

HX711_updateWeight(leftForkSensor,20);
ESP_LOGI("MAIN", "Current Force: %d", leftForkSensor->Weight);
vTaskDelay(pdMS_TO_TICKS(50));

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


}