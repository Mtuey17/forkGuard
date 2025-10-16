#include "mfrc522.h"
#include "driver/spi_master.h"
#include "esp_log.h"




void app_main(void)
{
    // initialize SPI bus (example for HSPI)
    spi_bus_config_t buscfg = {
        .miso_io_num = 19,
        .mosi_io_num = 23,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));

    // device config
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (you can increase)
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 1,
    };

    mfrc522_t dev;
    ESP_ERROR_CHECK(mfrc522_init(&dev, HSPI_HOST, &devcfg, 4));  // suppose reset is on GPIO4

    while (1) {
        uint8_t tagtype[2];
        size_t ttlen = sizeof(tagtype);
        if (mfrc522_request(&dev, 0x26 /* REQA */, tagtype, &ttlen) == ESP_OK) {
            ESP_LOGI("mainLoop", "Card detected, type: 0x%02x 0x%02x", tagtype[0], tagtype[1]);
            uint8_t uid[10];
            size_t uid_len = sizeof(uid);
            if (mfrc522_anticoll(&dev, uid, &uid_len) == ESP_OK) {
                ESP_LOGI("mainLoop", "UID:");
                for (int i = 0; i < uid_len; i++) {
                    ESP_LOGI("mainLoop", "  0x%02x", uid[i]);
                }
            } else {
                ESP_LOGW("mainLoop", "anticollision failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
