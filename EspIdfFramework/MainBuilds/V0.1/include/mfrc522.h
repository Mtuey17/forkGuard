

#ifndef MFRC522_H
#define MFRC522_H

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

// pins
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11 // 
#define PIN_NUM_CLK  12 // AKA SCK
#define PIN_NUM_CS   10  // AKA SDA
#define PIN_NUM_RST  18
#define LED_GREEN    8    // LED pin

// Initializes GPIO, reset pin, SPI, and MFRC522.
esp_err_t mfrc522_init(void);

// Try to read a 4-byte UID from a nearby card.
// Returns 0 on success (uid[0..3] filled), non-zero on error / no card.
uint8_t mfrc522_read_uid(uint8_t uid[4]);

#endif // MFRC522_H
