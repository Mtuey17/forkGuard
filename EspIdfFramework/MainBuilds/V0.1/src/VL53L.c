/* Matthew Tuer
   October 10th, 2025
   mtuer3727@conestogac.on.ca
   matthewjtuer@gmail.com
   C file for VL53L0X Time-of-Flight distance sensor
*/

#include <math.h>
#include "VL53L.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "VL53L0X";

VL53L_t *initializeTOF(i2c_master_bus_handle_t bus_handle, uint8_t id) {

    // 1. I2C device setup
    i2c_device_config_t TOF_CONFIG = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = id,
        .scl_speed_hz = 400000,
    };

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &TOF_CONFIG, &dev_handle);

    VL53L_t *VL53L_instance = malloc(sizeof(VL53L_t));
    VL53L_instance->dev_handle = dev_handle;
    VL53L_instance->deviceID = id;
    VL53L_instance->distance = 0;

    vTaskDelay(pdMS_TO_TICKS(1500));

    //vl53l0x_set_address(VL53L_instance,0x28);//to change adress (does not work)
    //VL53L_instance->deviceID = 0x28;


    uint8_t reg = 0xC0;
    uint8_t id_value = 0;
    i2c_master_transmit_receive(VL53L_instance->dev_handle, &reg, 1, &id_value, 1, -1);
    ESP_LOGI(TAG, "VL53L0X ID: 0x%02X", id_value);//logging id (mainly for debug purpouses)
    VL53L_instance->validID=id_value;



    i2c_master_transmit(VL53L_instance->dev_handle, (uint8_t[]){0x00, 0x02}, 2, pdMS_TO_TICKS(50)); // 0x02 continuous polling
    i2c_master_transmit(VL53L_instance->dev_handle, (uint8_t[]){0x04, 0x14}, 2, pdMS_TO_TICKS(50)); //set time interval to 20ms
    vTaskDelay(pdMS_TO_TICKS(50));
    return VL53L_instance;
    }

bool updateDistance(VL53L_t *TOF) {

    uint8_t response[2] = {0};

    if (i2c_master_transmit_receive(TOF->dev_handle, (uint8_t[]){0x1E}, 1, response, 2, -1) != ESP_OK) {//0x1E-result reg
        ESP_LOGW(TAG, "I2C timeout reading distance");
        return false;
    }
    uint16_t distance_raw = ((uint16_t)response[0] << 8) | response[1];
    TOF->distance = distance_raw / 10; // to cm
    return true;
}

bool vl53l0x_set_address(VL53L_t *TOF, uint8_t new_address) {//does not work
    if (!TOF) return false;

    uint8_t reg = 0x8A;//address register 
    uint8_t value = new_address & 0x7F;  

    // write new address
    if (i2c_master_transmit(TOF->dev_handle, (uint8_t[]){reg, value}, 2, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGW("VL53L0X", "Failed to write new I2C address");
        return false;
    }

    TOF->deviceID = new_address;
    ESP_LOGI("VL53L0X", "I2C address changed to 0x%02X", new_address);

    return true;
}

