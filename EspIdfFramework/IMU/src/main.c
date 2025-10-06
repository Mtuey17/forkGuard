#include "driver/i2c_master.h"
#include "IMU.h"


#define SDAPIN 15
#define SCLPIN 16

void app_main() {

    i2c_master_bus_config_t I2C_0_CONFIG = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCLPIN,
    .sda_io_num = SDAPIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&I2C_0_CONFIG, &bus_handle);


/*
i2c_device_config_t IMU_CONFIG = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x58,
    .scl_speed_hz = 100000,
};
i2c_master_dev_handle_t dev_handle;
i2c_master_bus_add_device(bus_handle, &IMU_CONFIG, &dev_handle);
//put into gyro only mode 
uint8_t gyro_mode[1] ={0x03};
uint8_t response[1];
//master handle, to send, size of to send, read size(bytes) timeout (-1=none)
i2c_master_transmit_receive(dev_handle, gyro_mode, sizeof(gyro_mode), response, 1, -1);
*/
    //page 60 of datasheet for id, maybe 28 0r 29
    IMU_t *IMU=initializeIMU(bus_handle,0x28);


    while(1){

        updateAngles(IMU);
        int xAngle=IMU->xAngle;


        //14-19
        /*
        uint8_t responseLSB[1];
        uint8_t responseMSB[1];
        uint8_t xLSB[1]={0x14};
        uint8_t xMSB[1]={0x15};
        int xAngle=0;
        i2c_master_transmit_receive(dev_handle, xLSB, sizeof(xLSB), responseLSB, 1, -1);
        i2c_master_transmit_receive(dev_handle, xMSB, sizeof(xMSB), responseMSB, 1, -1);
        xAngle=(responseMSB[0]<<8)|responseLSB[0];

        uint8_t yLSB[1]={0x16};
        uint8_t yMSB[1]={0x17};
        int yAngle=0;
        i2c_master_transmit_receive(dev_handle, yLSB, sizeof(yLSB), responseLSB, 1, -1);
        i2c_master_transmit_receive(dev_handle, yMSB, sizeof(yMSB), responseMSB, 1, -1);
        yAngle=(responseMSB[0]<<8)|responseLSB[0];

        uint8_t zLSB[1]={0x18};
        uint8_t zMSB[1]={0x19};
        int zAngle=0;
        i2c_master_transmit_receive(dev_handle, zLSB, sizeof(zLSB), responseLSB, 1, -1);
        i2c_master_transmit_receive(dev_handle, zMSB, sizeof(zMSB), responseMSB, 1, -1);
        zAngle=(responseMSB[0]<<8)|responseLSB[0];*/

    }


}