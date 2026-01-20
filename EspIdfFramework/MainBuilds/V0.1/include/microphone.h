#ifndef microphone_H
#define microphone_H

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define D0_PIN        GPIO_NUM_41   // Sound sensor digital output (DO)
#define RED_LED       GPIO_NUM_19   // RED LED pin

// Initialize GPIOs for the loud-noise detector
void microphoneInit(void);

// Loop that monitors the sensor and controls the LED
int microphoneLevel();

#ifdef __cplusplus
}
#endif

#endif
