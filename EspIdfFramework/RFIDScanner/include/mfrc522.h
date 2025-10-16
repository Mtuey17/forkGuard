#ifndef MFRC522_H
#define MFRC522_H

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>

typedef struct {
    spi_device_handle_t spi;
    int pin_reset;      // GPIO for NRSTPD / reset (optional, -1 if not used)
    // Possibly interrupt pin, etc.
    SemaphoreHandle_t lock;
} mfrc522_t;

esp_err_t mfrc522_init(mfrc522_t *dev,
                       spi_host_device_t host,
                       const spi_device_interface_config_t *spi_cfg,
                       int pin_reset);

void mfrc522_reset(mfrc522_t *dev);

esp_err_t mfrc522_write_reg(mfrc522_t *dev, uint8_t addr, uint8_t val);
esp_err_t mfrc522_read_reg(mfrc522_t *dev, uint8_t addr, uint8_t *val);
esp_err_t mfrc522_write_fifo(mfrc522_t *dev, const uint8_t *data, size_t len);
esp_err_t mfrc522_read_fifo(mfrc522_t *dev, uint8_t *data, size_t len);

esp_err_t mfrc522_set_bitmask(mfrc522_t *dev, uint8_t addr, uint8_t mask);
esp_err_t mfrc522_clear_bitmask(mfrc522_t *dev, uint8_t addr, uint8_t mask);

/**
 * Send a command to MFRC522 along with data and read back response.
 * cmd: command (e.g. PCD_TRANSCEIVE, PCD_AUTHENT, etc.)
 * send: buffer to send
 * send_len: length of send buffer
 * recv: output buffer for received data (if any)
 * recv_max_len: max length we can receive
 * recv_len: actual length received
 * valid_bits: for receiving bits not aligned on byte boundary (0 if none)
 * timeout_ms: how long to wait
 */
esp_err_t mfrc522_transceive(mfrc522_t *dev,
                             uint8_t cmd,
                             const uint8_t *send,
                             size_t send_len,
                             uint8_t *recv,
                             size_t recv_max_len,
                             size_t *recv_len,
                             uint8_t *valid_bits,
                             uint32_t timeout_ms);

// High-level convenience:
esp_err_t mfrc522_request(mfrc522_t *dev, uint8_t req_mode, uint8_t *resp, size_t *resp_len);
esp_err_t mfrc522_anticoll(mfrc522_t *dev, uint8_t *uid_out, size_t *uid_len);

#endif // MFRC522_H
