#include "RTLS.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <stdio.h>
#include "esp_log.h"


RTLS_Instance  *RTLS_init(uart_port_t uart_num, uint8_t Rx,uint8_t Tx, uint32_t baudRate){


   const uart_port_t RTLS_PORT = uart_num;
    uart_config_t uart_config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    uart_param_config(RTLS_PORT, &uart_config);
    uart_set_pin(RTLS_PORT, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(RTLS_PORT, 1024, 2048, 0, NULL, 0);

    RTLS_Instance *RTLS_I = malloc(sizeof(RTLS_Instance));
    RTLS_I->uart_num = uart_num;
    RTLS_I->anchorId=0;
    RTLS_I->previousId=0;
    RTLS_I->sendRateHz=1.0f;
    RTLS_I->lastSendTime=0;
    return RTLS_I;

}


bool UPDATE_LOCATION(RTLS_Instance *RT_Instance, uint16_t timeout_ms)
{
    vTaskDelay(pdMS_TO_TICKS(2));

    int startTime = esp_timer_get_time() / 1000;
    size_t incomingDataLength = 0;

    while (incomingDataLength < 7) {
        uart_get_buffered_data_len(RT_Instance->uart_num, &incomingDataLength);

        int currentTime = esp_timer_get_time() / 1000;
        if ((currentTime - startTime) > timeout_ms) {
            ESP_LOGI("RTLS", "Device Timeout");
            return false;
        }
    }

    char buf[8] = {0};
    int bytesRead = uart_read_bytes(RT_Instance->uart_num, (uint8_t *)buf, 7, pdMS_TO_TICKS(5));

    if (bytesRead != 7) {
        ESP_LOGI("RTLS", "Read Error");
        return false;
    }

    if (strncmp(buf, "Anchor", 6) == 0 && buf[6] >= '0' && buf[6] <= '9') {
        RT_Instance->anchorId = buf[6] - '0';
        return true;
    }
    uart_flush(RT_Instance->uart_num);
    ESP_LOGI("RTLS", "Invalid Data: %s", buf);
    return false;
}
