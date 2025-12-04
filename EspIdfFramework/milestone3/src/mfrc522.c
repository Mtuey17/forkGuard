
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#include "mfrc522.h"

// MFRC522 registers
#define CommandReg      0x01 // starts and stops command execution
#define ComIEnReg       0x02 // enable and disable interrupt request control bits
#define ComIrqReg       0x04 // interrupt request bits
#define ErrorReg        0x06 // error bits showing the error status of the last command executed
#define FIFODataReg     0x09 // input and output of 64 byte FIFO buffer
#define FIFOLevelReg    0x0A // number of bytes stored in the FIFO buffer
#define ControlReg      0x0C // miscellaneous control registers
#define BitFramingReg   0x0D // adjustments for bit-oriented frames
#define CollReg         0x0E // bit position of the first bit-collision detected on the RF interface
#define ModeReg         0x11 // defines general modes for transmitting and receiving
#define TxControlReg    0x14 // controls the logical behavior of the antenna driver pins TX1 and TX2
#define TxASKReg        0x15 // controls the setting of the transmission modulation
#define RFCfgReg        0x26 // configures the receiver gain
#define TModeReg        0x2A // defines settings for the internal timer
#define TPrescalerReg   0x2B // defines settings for the internal timer
#define TReloadRegH     0x2C // defines the 16-bit timer reload value (HIGH)
#define TReloadRegL     0x2D // defines the 16-bit timer reload value (LOW)


// PCD (Proximity Coupling Device) commands
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_RESETPHASE  0x0F

// PICC (Proximity Integrated Circuit Card) commands
#define PICC_REQIDL     0x26 // REQA (7-bit)
#define PICC_ANTICOLL   0x93 // Anticollision CL1

static const char *TAG = "RC522_UID";
static spi_device_handle_t spiDevice;

// led gpio init
static void gpio_init(void)
{
// LED as output (start OFF)
    gpio_config_t io_led = {
        .pin_bit_mask = (1ULL << LED_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_led);
    gpio_set_level(LED_GREEN, 0);
}

// SPI helpers (address[7] = R/W, [6..1] = reg, [0] = 0)
static inline uint8_t writeAddress(uint8_t r){
    return (r<<1) & 0x7E;
}
static inline uint8_t readAddress(uint8_t r){
    return ((r<<1)&0x7E) | 0x80;
}

// Reads register and returns value
static esp_err_t readRegister(uint8_t reg, uint8_t *val){
    uint8_t tx[2] = { readAddress(reg), 0x00 }; // sets where to read from
    uint8_t rx[2] = {0}; 
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    ESP_RETURN_ON_ERROR(spi_device_transmit(spiDevice, &t), TAG, "spi rx"); // ESP return function if ESP not OK
    *val = rx[1]; return ESP_OK; // otherwise, sets value to read value and returns ESP_OK
}

// Writes to a register
static esp_err_t writeRegister(uint8_t reg, uint8_t val){
    uint8_t tx[2] = { writeAddress(reg), val }; // sets where to write and what
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(spiDevice, &t); // returns ESP_OK if completed with no issues
}

// soft reset mfrc522 chip incase of unwanted values in registers
static void mfrc522Reset(void){
    writeRegister(CommandReg, PCD_RESETPHASE);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Enables Antenna for RFID Reader
static void mfrc522AntennaOn(void){
    uint8_t v=0; 
    readRegister(TxControlReg, &v);
    if ((v & 0x03) != 0x03) { // checks if on and if not on, turn on
        writeRegister(TxControlReg, v | 0x03);
    }
}

// initializes mfrc522
static void mfrc522Initialize(void){
    mfrc522Reset();
    writeRegister(TModeReg,      0x8D); // timer, timemout mode, and prescaler (HIGHER 4 bits)
    writeRegister(TPrescalerReg, 0x3E); // prescaler (LOWER 8 bits)
    writeRegister(TReloadRegL,   30);   // reload timeout value:
    writeRegister(TReloadRegH,   0);    // at freq. 2kHz, timeout is about 15 ms
    writeRegister(TxASKReg,      0x40); // 100% ASK to meet PICC ISO standard
    writeRegister(ModeReg,       0x3D); // CRC preset 0x6363
    writeRegister(RFCfgReg,      0x70); // max RX gain to pick up cards easier
    mfrc522AntennaOn();
}

// Exchange data with PICC
static uint8_t toCard(uint8_t command, const uint8_t *send, 
        uint8_t sendLen, uint8_t *back, uint8_t *backBits){
    
    uint8_t irqEn=0;
    uint8_t waitIrq=0;
    uint8_t n=0;
    
    // setup IRQ (Interrupt Request) and FIFO buffer
    if (command == PCD_TRANSCEIVE) { 
        irqEn = 0x77; 
        waitIrq = 0x30;
    } 
    writeRegister(ComIEnReg, irqEn | 0x80);
    writeRegister(ComIrqReg, 0x7F);
    writeRegister(FIFOLevelReg, 0x80);
    writeRegister(CommandReg, PCD_IDLE);

    // fills sends bytes into buffer
    for (uint8_t i=0;i<sendLen;i++) {
        writeRegister(FIFODataReg, send[i]);
    }

    // starts command
    writeRegister(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        uint8_t bf; 
        readRegister(BitFramingReg, &bf);
        writeRegister(BitFramingReg, bf | 0x80); // StartSend
    }

    // waits for completeion or timeout signal
    int tries = 2000;
    do { 
        readRegister(ComIrqReg, &n); 
        tries--; 
    } while (tries && !(n & 0x01) && !(n & waitIrq));

    uint8_t bf; 
    readRegister(BitFramingReg, &bf);
    writeRegister(BitFramingReg, bf & (uint8_t)~0x80); // clear StartSend

    // check if timout or error occurred
    if (!tries) return 1; // timeout
    uint8_t err; 
    readRegister(ErrorReg, &err);
    if (err & 0x1B) return 2; // BufferOvfl|ParityErr|ProtocolErr

    // Read reply from PICC
    if (command == PCD_TRANSCEIVE && back && backBits) {
        uint8_t fifo=0;
        uint8_t lastBits=0;
        readRegister(FIFOLevelReg, &fifo);
        readRegister(ControlReg, &lastBits); 
        lastBits &= 0x07;
        
        if (lastBits) {
            *backBits = (uint8_t)((fifo - 1U)*8U + lastBits);
        }
        else {
            *backBits = (uint8_t)(fifo * 8U);
        }

        if (fifo > 16) {
            fifo = 16;
        }

        for (uint8_t i=0;i<fifo;i++) {
            readRegister(FIFODataReg, &back[i]);
        }
    }
    return 0;
}

// REQA (Request, Type-A) an ISO Standard for our PICC
// Looking for a card nearby
static uint8_t reqa(uint8_t *atqa){
    uint8_t bits=0;
    writeRegister(CollReg, 0x80); // setup for anticollision later
    writeRegister(BitFramingReg, 0x07); // REQA is a 7-bit short frame
    atqa[0] = PICC_REQIDL; // loads REQA code to ATQA (Answer To Request, Type A) ISO standard
    if(toCard(PCD_TRANSCEIVE, atqa, 1, atqa, &bits) == 0 && bits == 16){
        return 0;
    }
    return 1;
}

// Anticollision (CL1) — returns 4 UID bytes + BCC (5 bytes)
static uint8_t anticoll_cl1(uint8_t serNum[5]){
    uint8_t frame[2] = { PICC_ANTICOLL, 0x20 }; // fills frame for tx with Anticollision code
    uint8_t bits=0;
    writeRegister(BitFramingReg, 0x00); // byte aligned mode
    if (toCard(PCD_TRANSCEIVE, frame, 2, serNum, &bits) != 0) return 1;
    if (bits != 40) return 2;  // expect 5 bytes
    uint8_t bcc = serNum[0]^serNum[1]^serNum[2]^serNum[3]; // calculates bcc (XOR of first four bytes)
    if (bcc == serNum[4]){ // confirms actual bcc matches calculated
        return 0;
    }
    return 3;
}

// ESP32 SPI setup, adding to config
static esp_err_t spiInitialize(void){
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED), TAG, "bus"); // ESP return error if not OK function
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000*1000,  // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    return spi_bus_add_device(SPI3_HOST, &devcfg, &spiDevice);
}

// Public init wrapper that uses your original init code
esp_err_t mfrc522_init(void)
{
    // Reset pin
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };

    
    // initialization
    gpio_init();

    gpio_config(&io);
    gpio_set_level(PIN_NUM_RST, 0); 
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1); 
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);

    esp_err_t err = spiInitialize();
    if (err != ESP_OK) {
        return err;
    }

    mfrc522Initialize();

    return ESP_OK;
}

// Non-blocking helper that performs REQA + anticollision and returns 4-byte UID
uint8_t mfrc522_read_uid(uint8_t uid[4])
{
    uint8_t atqa[2] = {0};
    if (reqa(atqa) != 0) {
        return 1; // no card / REQA failed
    }

    uint8_t uid_bcc[5] = {0};
    if (anticoll_cl1(uid_bcc) != 0) {
        return 2; // anticollision / BCC error
    }

    uid[0] = uid_bcc[0];
    uid[1] = uid_bcc[1];
    uid[2] = uid_bcc[2];
    uid[3] = uid_bcc[3];

    return 0;
}
