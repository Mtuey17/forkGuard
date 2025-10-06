#include "IMU.h"
#include "driver/i2c_master.h"  


IMU_t *initializeIMU(i2c_master_bus_handle_t bus_handle, uint8_t id){

    i2c_device_config_t IMU_CONFIG = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = id,
    .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &IMU_CONFIG, &dev_handle);
    IMU_t *imu_instance = malloc(sizeof(IMU_t));
    imu_instance->dev_handle=dev_handle;
    imu_instance->deviceID=id;
    imu_instance->xAngle=0;
    imu_instance->yAngle=0;
    imu_instance->zAngle=0;



    //if read size 0, automatically a write
    uint8_t configMode[2] = {0x3D, 0x00}; //3D OPR_MODE, 0x00=config mode 
    i2c_master_transmit_receive(imu_instance->dev_handle, configMode, 2, NULL, 0, -1);//handler, data to send, data length, read variable, read length, timeout

    uint8_t pwrMode[2] = {0x3E, 0x00}; // 0x3E = PWR_MODE register, 0x00 = Normal power mode
    i2c_master_transmit_receive(imu_instance->dev_handle, pwrMode, 2, NULL, 0, -1);//handler, data to send, data length, read variable, read length, timeout

    uint8_t buf[2] = {0x3D, 0x03};  // 0x3D = OPR_MODE register, 0x03 = gyroonly
    i2c_master_transmit_receive(imu_instance->dev_handle, buf, 2, NULL, 0, -1);//handler, data to send, data length, read variable, read length, timeout

    return imu_instance;
}

void updateAngles(IMU_t *IMU){
    uint8_t responseLSB[1];
    uint8_t responseMSB[1];

    uint8_t xLSB[1]={0x14};
    uint8_t xMSB[1]={0x15};
    //master handle, to send, size of to send, read size(bytes) timeout (-1=none)
    i2c_master_transmit_receive(IMU->dev_handle, xLSB, sizeof(xLSB), responseLSB, 1, -1);
    i2c_master_transmit_receive(IMU->dev_handle, xMSB, sizeof(xMSB), responseMSB, 1, -1);
    IMU->xAngle=(int16_t)((responseMSB[0]<<8)|responseLSB[0]);

    uint8_t yLSB[1]={0x16};
    uint8_t yMSB[1]={0x17};
    i2c_master_transmit_receive(IMU->dev_handle, yLSB, sizeof(yLSB), responseLSB, 1, -1);
    i2c_master_transmit_receive(IMU->dev_handle, yMSB, sizeof(yMSB), responseMSB, 1, -1);
    IMU->yAngle=(int16_t)((responseMSB[0]<<8)|responseLSB[0]);

    uint8_t zLSB[1]={0x18};
    uint8_t zMSB[1]={0x19};
    i2c_master_transmit_receive(IMU->dev_handle, zLSB, sizeof(zLSB), responseLSB, 1, -1);
    i2c_master_transmit_receive(IMU->dev_handle, zMSB, sizeof(zMSB), responseMSB, 1, -1);
    IMU->zAngle=(int16_t)((responseMSB[0]<<8)|responseLSB[0]);

}