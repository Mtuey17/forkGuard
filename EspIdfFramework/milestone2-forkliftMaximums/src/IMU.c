/* Matthew Tuer 
october 10th, 2025
mtuer3727@conestogac.on.ca
matthewjtuer@gmail.com 
c file for BNO055 IMU
*/


#include <math.h>
#include "IMU.h"
#include "driver/i2c_master.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


IMU_t *initializeIMU(i2c_master_bus_handle_t bus_handle, uint8_t id){

    //i2c setup 
    i2c_device_config_t IMU_CONFIG = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = id,
    .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &IMU_CONFIG, &dev_handle);
    IMU_t *imu_instance = malloc(sizeof(IMU_t));
    imu_instance->dev_handle=dev_handle;
    imu_instance->deviceID=id;
    imu_instance->heading=0;
    imu_instance->roll=0;
    imu_instance->pitch=0;
    imu_instance->forwardAcceleration=0;
    imu_instance->rawYAcceleration=0;
    imu_instance->rawZAcceleration=0;



    // 1. CONFIGMODE
    vTaskDelay(pdMS_TO_TICKS(2500));
        uint8_t configMode[2] = {0x3D, 0x00};
        i2c_master_transmit(dev_handle, configMode, 2, pdMS_TO_TICKS(150));
        vTaskDelay(pdMS_TO_TICKS(50));

        // 2. Power mode
        uint8_t pwrMode[2] = {0x3E, 0x00};
        i2c_master_transmit(dev_handle, pwrMode, 2, pdMS_TO_TICKS(150));
        vTaskDelay(pdMS_TO_TICKS(50));

        // 3. Units (Euler angles in degrees, accel in m/s^2)
        uint8_t units[2] = {0x3B, 0x00};
        i2c_master_transmit(dev_handle, units, 2, pdMS_TO_TICKS(150));
        vTaskDelay(pdMS_TO_TICKS(50));

        // 4. Operation mode: NDOF
        uint8_t opMode[2] = {0x3D, 0x0C};
        i2c_master_transmit(dev_handle, opMode, 2, pdMS_TO_TICKS(150));
        vTaskDelay(pdMS_TO_TICKS(500)); // wait for fusion to stabilize



    return imu_instance;
}


bool IMUInfo(IMU_t*IMU){

    

    ESP_LOGI("Info", "**IMU INFO**");

    //chip id
    uint8_t reg = 0x00;
    uint8_t id2 = 0;
    i2c_master_transmit_receive(IMU->dev_handle, &reg, 1, &id2, 1, -1);
    ESP_LOGI("Info", "Chip ID: 0x%02X", id2);

    //mode
    reg = 0x3D;
    uint8_t mode2 = 0;
    i2c_master_transmit_receive(IMU->dev_handle, &reg, 1, &mode2, 1, -1);
    ESP_LOGI("Info", "OPR_MODE = 0x%02X", mode2);


    ESP_LOGI("Info", "-----------");
    return true;

}
bool updateAngles(IMU_t *IMU){

    uint8_t response[2];

    //master handle, to send, size of to send, read size(bytes) timeout (-1=none)
    uint8_t eulerX[2]={0x1A,0x1B};
    i2c_master_transmit_receive(IMU->dev_handle, eulerX, sizeof(eulerX), response, 2, -1);
    IMU->heading = ((int16_t)((response[1] << 8) | response[0])) / 16;

    uint8_t eulerY[2]={0x1C,0x1D};
    i2c_master_transmit_receive(IMU->dev_handle, eulerY, sizeof(eulerY), response, 2, -1);
    IMU->roll=((int16_t)((response[1] << 8) | response[0])) / 16;
 
    uint8_t eulerZ[2]={0x1E,0x1F};
    i2c_master_transmit_receive(IMU->dev_handle, eulerZ, sizeof(eulerZ), response, 2, -1);
    IMU->pitch=((int16_t)((response[1] << 8) | response[0])) / 16;
    return true;

}


bool updateAcceleration(IMU_t* IMU){

    //master handle, to send, size of to send, read size(bytes) timeout (-1=none)
    uint8_t response[2];
    uint8_t accelY[2]={0x0A,0x0B};
    i2c_master_transmit_receive(IMU->dev_handle, accelY, sizeof(accelY), response, 2, -1);
    int16_t raw_acc = (int16_t)((response[1] << 8) | response[0]);
    IMU->rawYAcceleration = (float)raw_acc / 100.0f;

    uint8_t accelZ[2]={0x0C,0x0D};
    i2c_master_transmit_receive(IMU->dev_handle, accelZ, sizeof(accelZ), response, 2, -1);
    raw_acc = (int16_t)((response[1] << 8) | response[0]);
    IMU->rawZAcceleration = (float)raw_acc / 100.0f;


 
    float radianPitch=IMU->pitch* (M_PI / 180.0f);
    float forwardAcc = (IMU->rawYAcceleration * cosf(radianPitch) + IMU->rawZAcceleration * sinf(radianPitch))*-1;
    
    IMU->forwardAcceleration=forwardAcc;


    return true;

}