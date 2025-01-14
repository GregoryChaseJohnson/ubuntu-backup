#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "camera_pins.h"

#define CONFIG_XCLK_FREQ 20000000
#define CAPTURE_DELAY_MS 200

// Camera configuration for ESP32-CAM AI-THINKER model
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = CONFIG_XCLK_FREQ,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_UXGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY // Added for consistency
};
#define PIR_SENSOR_GPIO GPIO_NUM_13 // PIR sensor GPIO

static const char* TAG = "esp32_camera";

void capture_and_save(const char* base_path, uint32_t duration_ms) {
    uint32_t start_time = esp_timer_get_time() / 1000; // Get start time in milliseconds
    uint32_t elapsed_time = 0;
    uint32_t image_count = 0;

    while (elapsed_time < duration_ms) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            char filepath[64];
            snprintf(filepath, sizeof(filepath), "%s/image_%lu.jpg", base_path, image_count++);
            FILE* file = fopen(filepath, "wb");
            if (file) {
                fwrite(fb->buf, 1, fb->len, file);
                fclose(file);
                vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));
            } else {
                ESP_LOGE(TAG, "Failed to open file for writing");
            }
            esp_camera_fb_return(fb);
        } else {
            ESP_LOGE(TAG, "Camera capture failed");
        }
        // Update elapsed time
        elapsed_time = (esp_timer_get_time() / 1000) - start_time;
        // Delay to allow other tasks to run and control capture rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() {
    // NVS Initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Camera Initialization
    ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    // SD Card Initialization (using SDMMC)
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = "/sdcard";
    ESP_LOGI(TAG, "Initializing SD card");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: (%s)", esp_err_to_name(ret));
        return;
    }

    // PIR Sensor Setup
    gpio_set_direction(PIR_SENSOR_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(PIR_SENSOR_GPIO); // Optionally enable pull-up

    ESP_LOGI(TAG, "System ready, monitoring for motion.");

    while (true) {
        if (gpio_get_level(PIR_SENSOR_GPIO) == 1) { // Motion detected
            ESP_LOGI(TAG, "Motion detected, capturing images.");
            capture_and_save("/sdcard", 7000); // Capture for 7 seconds
            ESP_LOGI(TAG, "Capture complete. System returning to motion detection mode.");
            // Delay to avoid immediate re-triggering
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        // Sleep to conserve energy between checks
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
