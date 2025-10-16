#include "mfrc522.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "mfrc522";

// From MFRC522 datasheet: SPI read/write addressing: MSB = 1 for read, 0 for write; bits 6:1 are address, bit 0 is 0 (for alignment)  
#define MFRC522_ADDR_READ(addr)  ( (addr << 1) & 0x7E ) | 0x80
#define MFRC522_ADDR_WRITE(addr) ( (addr << 1) & 0x7E )

// MFRC522 command set (subset)
#define PCD_IDLE         0x00
#define PCD_MEM          0x01
#define PCD_GENERATE_RANDOM_ID 0x02
#define PCD_CALCCRC      0x03
#define PCD_TRANSMIT     0x04
#define PCD_NO_CMD_CHANGE 0x07
#define PCD_RECEIVE      0x08
#define PCD_TRANSCEIVE   0x0C
#define PCD_AUTHENT      0x0E
#define PCD_SOFTRESET    0x0F

// MFRC522 registers (subset). You will need more from datasheet.
enum {
    // Page 0: Command and status
    CommandReg       = 0x01,
    ComIEnReg        = 0x02,
    DivIEnReg        = 0x03,
    ComIrqReg        = 0x04,
    DivIrqReg        = 0x05,
    ErrorReg         = 0x06,
    Status1Reg       = 0x07,
    Status2Reg       = 0x08,
    FIFODataReg      = 0x09,
    FIFOLevelReg     = 0x0A,
    ControlReg       = 0x0C,
    BitFramingReg    = 0x0D,
    CollReg          = 0x0E,
    // Page 1: Configuration
    ModeReg          = 0x11,
    TxModeReg        = 0x12,
    RxModeReg        = 0x13,
    TModeReg         = 0x2A,
    TPrescalerReg    = 0x2B,
    TReloadRegH      = 0x2C,
    TReloadRegL      = 0x2D,
    TxControlReg     = 0x14,
    // CRC
    CRCResultRegH    = 0x21,
    CRCResultRegL    = 0x22,
    // etc...
};

// Internal: select SPI and lock
static esp_err_t mfrc522_spi_transfer(mfrc522_t *dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = { 0 };
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    // No flags like SPI_TRANS_USE_TXDATA etc, use normal buffer
    return spi_device_transmit(dev->spi, &t);
}

esp_err_t mfrc522_init(mfrc522_t *dev,
                       spi_host_device_t host,
                       const spi_device_interface_config_t *spi_cfg,
                       int pin_reset)
{
    if (!dev || !spi_cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    dev->pin_reset = pin_reset;
    dev->lock = xSemaphoreCreateMutex();
    if (dev->lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // Add device to SPI bus
    esp_err_t ret = spi_bus_add_device(host, spi_cfg, &dev->spi);
    if (ret != ESP_OK) {
        return ret;
    }
    // Reset device if pin provided
    if (pin_reset >= 0) {
        gpio_set_direction(pin_reset, GPIO_MODE_OUTPUT);
        // Pulse low, then high
        gpio_set_level(pin_reset, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(pin_reset, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Soft reset
    mfrc522_reset(dev);

    // Timer, mode, antenna on, etc initialization
    // Example: Turn antenna on by setting TxControlReg bit 0x03
    mfrc522_write_reg(dev, TxControlReg, 0x83);

    return ESP_OK;
}

void mfrc522_reset(mfrc522_t *dev)
{
    // send soft reset
    mfrc522_write_reg(dev, CommandReg, PCD_SOFTRESET);
    // According to datasheet, wait some time until reset done
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Write one register
esp_err_t mfrc522_write_reg(mfrc522_t *dev, uint8_t addr, uint8_t val)
{
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t tx[2];
    tx[0] = MFRC522_ADDR_WRITE(addr);
    tx[1] = val;
    esp_err_t err = mfrc522_spi_transfer(dev, tx, NULL, 2);
    xSemaphoreGive(dev->lock);
    return err;
}

// Read one register
esp_err_t mfrc522_read_reg(mfrc522_t *dev, uint8_t addr, uint8_t *val)
{
    if (!val) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = MFRC522_ADDR_READ(addr);
    tx[1] = 0x00;
    esp_err_t err = mfrc522_spi_transfer(dev, tx, rx, 2);
    if (err == ESP_OK) {
        *val = rx[1];
    }
    xSemaphoreGive(dev->lock);
    return err;
}

// Write FIFO buffer
esp_err_t mfrc522_write_fifo(mfrc522_t *dev, const uint8_t *data, size_t len)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    // first send address for FIFODataReg write
    uint8_t addr = MFRC522_ADDR_WRITE(FIFODataReg);
    // We can send addr + data in one transaction
    uint8_t *tx = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    if (!tx) {
        xSemaphoreGive(dev->lock);
        return ESP_ERR_NO_MEM;
    }
    tx[0] = addr;
    memcpy(&tx[1], data, len);
    esp_err_t err = mfrc522_spi_transfer(dev, tx, NULL, len + 1);
    heap_caps_free(tx);
    xSemaphoreGive(dev->lock);
    return err;
}

// Read FIFO buffer
esp_err_t mfrc522_read_fifo(mfrc522_t *dev, uint8_t *data, size_t len)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t *tx = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    uint8_t *rx = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    if (!tx || !rx) {
        if (tx) heap_caps_free(tx);
        if (rx) heap_caps_free(rx);
        xSemaphoreGive(dev->lock);
        return ESP_ERR_NO_MEM;
    }
    tx[0] = MFRC522_ADDR_READ(FIFODataReg);
    for (size_t i = 1; i <= len; i++) {
        tx[i] = 0x00;
    }
    esp_err_t err = mfrc522_spi_transfer(dev, tx, rx, len + 1);
    if (err == ESP_OK) {
        memcpy(data, &rx[1], len);
    }
    heap_caps_free(tx);
    heap_caps_free(rx);
    xSemaphoreGive(dev->lock);
    return err;
}

esp_err_t mfrc522_set_bitmask(mfrc522_t *dev, uint8_t addr, uint8_t mask)
{
    uint8_t cur;
    esp_err_t err = mfrc522_read_reg(dev, addr, &cur);
    if (err != ESP_OK) return err;
    return mfrc522_write_reg(dev, addr, cur | mask);
}

esp_err_t mfrc522_clear_bitmask(mfrc522_t *dev, uint8_t addr, uint8_t mask)
{
    uint8_t cur;
    esp_err_t err = mfrc522_read_reg(dev, addr, &cur);
    if (err != ESP_OK) return err;
    return mfrc522_write_reg(dev, addr, cur & (~mask));
}

esp_err_t mfrc523_transceive_poll(mfrc522_t *dev,
                                  const uint8_t *send,
                                  size_t send_len,
                                  uint8_t *recv,
                                  size_t recv_max_len,
                                  size_t *recv_len,
                                  uint8_t *valid_bits,
                                  uint32_t timeout_ms)
{
    // (Optional) you can split this internal logic; below is a simplified approach

    // Flush FIFO
    ESP_ERROR_CHECK(mfrc522_set_bitmask(dev, FIFOLevelReg, 0x80));  // flush FIFO
    ESP_ERROR_CHECK(mfrc522_clear_bitmask(dev, ComIrqReg, 0x80));   // clear all interrupts
    ESP_ERROR_CHECK(mfrc522_write_reg(dev, CommandReg, PCD_IDLE));

    // Write data to FIFO
    ESP_ERROR_CHECK(mfrc522_write_fifo(dev, send, send_len));

    // Set bit framing: no bits left over
    ESP_ERROR_CHECK(mfrc522_write_reg(dev, BitFramingReg, 0x00));

    // Execute transceive
    ESP_ERROR_CHECK(mfrc522_write_reg(dev, CommandReg, PCD_TRANSCEIVE));
    // StartSend = 1
    ESP_ERROR_CHECK(mfrc522_set_bitmask(dev, BitFramingReg, 0x80));

    // Wait for data or timeout
    uint32_t t0 = xTaskGetTickCount();
    uint8_t irq;
    while (1) {
        ESP_ERROR_CHECK(mfrc522_read_reg(dev, ComIrqReg, &irq));
        // Check RxIRq or Timeout
        if (irq & 0x30) break;  // RxIRq (0x10) or IdleIRq (0x20)
        if ((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
    }

    // Check error
    uint8_t err_reg;
    ESP_ERROR_CHECK(mfrc522_read_reg(dev, ErrorReg, &err_reg));
    if (err_reg & 0x1B) {
        // BufferOvfl, ParityErr, ProtocolErr etc
        return ESP_FAIL;
    }

    // Read number of bytes in FIFO
    uint8_t fifo_used;
    ESP_ERROR_CHECK(mfrc522_read_reg(dev, FIFOLevelReg, &fifo_used));
    if (fifo_used > recv_max_len) {
        // too much data
        return ESP_ERR_NO_MEM;
    }

    // Read data
    ESP_ERROR_CHECK(mfrc522_read_fifo(dev, recv, fifo_used));
    *recv_len = fifo_used;
    if (valid_bits) {
        // no bit-framing in this simple version
        *valid_bits = 0;
    }

    return ESP_OK;
}

esp_err_t mfrc522_request(mfrc522_t *dev, uint8_t req_mode, uint8_t *resp, size_t *resp_len)
{
    uint8_t cmd = req_mode;
    return mfrc523_transceive_poll(dev, &cmd, 1, resp, *resp_len, resp_len, NULL, 200);
}

esp_err_t mfrc522_anticoll(mfrc522_t *dev, uint8_t *uid_out, size_t *uid_len)
{
    uint8_t anticoll_cmd = 0x93;  // cascade level 1
    uint8_t buf[2];
    buf[0] = anticoll_cmd;
    buf[1] = 0x20;  // NVB = 0x20 means 32 bits (4 bytes)  
    size_t recv_len = *uid_len;
    esp_err_t err = mfrc523_transceive_poll(dev, buf, 2, uid_out, recv_len, &recv_len, NULL, 200);
    if (err != ESP_OK) return err;
    *uid_len = recv_len;
    return ESP_OK;
}
