#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"

// Pins
#define PIN_SCK GPIO_NUM_17
#define PIN_WS  GPIO_NUM_18
#define PIN_SD  GPIO_NUM_8

// Audio settings ~21.3ms per frame ... frameSamples/sampleRate
#define sampleRateHz    48000 // number of samples per second
#define frameSamples    1024 // Samples per frame

// dB conversion (approx)
// 94 dB SPL @ 1kHz ≈ -26 dBFS (PEAK). RMS sine is 3.01 dB below peak.
#define dBOffset    123.0103f
#define calibrateDB      0.0f // If we get hold of a SPL meter, we can calibrate with this. if low by 2, then add 2, and opposite

// Warning thresholds
#define warningSustainedDB 85.0f
#define sustainSecs  3
#define warningInstantDB 100.0f

static const char *TAG = "mic";
static i2s_chan_handle_t rx_chan;

// IMPORTANT: static buffer (not on task stack)
static int32_t buf[frameSamples * 2]; // stereo interleaved

static float dbLevel(const int32_t *stereo, int n)
{
    double sumSquare = 0.0;
    int count = 0;

    // Mic sends data left, right, left, right. This reads only lefts since L/R is strapped to ground
    for (int i = 0; i + 1 < n; i += 2) {
        // Common alignment: 24-bit data in top bits of 32-bit slot
        int32_t s24 = stereo[i] >> 8;
        float normalized = (float)s24 / (float)(1 << 23);  // normalizes 24 bits to ~-[1,1]

        sumSquare += (double)normalized * (double)normalized;
        count++; // number of samples in frame
    }

    if (count == 0){
        return -INFINITY;
    } 

    float meanSquare = (float)(sumSquare / (double)count);
    float rms = sqrtf(fmaxf(meanSquare, 1e-20f)); // square roots bigger of two to prevent sqrt(0) and log10(0)
    float dbfs_rms = 20.0f * log10f(rms);

    return dbfs_rms + dBOffset + calibrateDB;
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    // I2S RX channel (master)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // Standard I2S, stereo, 32-bit slots
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sampleRateHz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_SCK,
            .ws   = PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_SD,
            .invert_flags = { 0 },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    // How many frames ≈ 1 second?
    const int framesPerSec = sampleRateHz / frameSamples;
    int framesOver85 = 0;

    size_t bytesRead = 0;

    while (1) {
        ESP_ERROR_CHECK(i2s_channel_read(rx_chan, buf, sizeof(buf), &bytesRead, portMAX_DELAY));
        int n = bytesRead / sizeof(int32_t);

        float db = dbLevel(buf, n);

        // Print every frame (~46 times/sec)
        ESP_LOGI(TAG, "dB ≈ %.1f", db);

        // Instant warning
        if (db >= warningInstantDB) {
            ESP_LOGW(TAG, "INSTANT HIGH dB LEVEL WARNING! (>= %.0f dB)", warningInstantDB);
        }

        // Sustained warning (>=85 for ~3 seconds)
        if (db >= warningSustainedDB) {
            framesOver85++;
            if (framesOver85 >= framesPerSec * sustainSecs) {
                ESP_LOGW(TAG, "SUSTAINED HIGH dB LEVELS WARNING! (>= %.0f dB for %d s)", warningSustainedDB, sustainSecs);
                framesOver85 = 0; // reset so it doesn't spam every frame
            }
        } else {
            framesOver85 = 0;
        }
    }
}