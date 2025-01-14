#include "esp_camera.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "esp_task_wdt.h"
#include <esp_system.h>
#include "img_proc.h"
#include "esp_log.h"

static const char* TAG = "esp32-cam";

#define CONFIG_XCLK_FREQ 20000000 

//#define MAX_JPEG_BUFFER_SIZE 60000
//static bool is_first_frame = true;  // Initialize the flag

//static uint8_t out_img_buf[MAX_JPEG_BUFFER_SIZE];

static esp_err_t init_camera(void) {
    camera_config_t camera_config = {
        .pin_pwdn  = CAM_PIN_PWDN,
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
        .frame_size = FRAMESIZE_VGA,

        .jpeg_quality = 10,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

/*void convertToGreyscale(uint16_t* input_buf, uint8_t* output_buf, int width, int height);

void image_processing_task(uint8_t* jpg_buf, size_t jpg_buf_len) {
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = jpg_buf,
        .indata_size = jpg_buf_len,
        .outbuf = (uint8_t*)malloc(MAX_JPEG_BUFFER_SIZE), // Allocate memory for output buffer
        .outbuf_size = MAX_JPEG_BUFFER_SIZE,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_1_8,
        .flags = { .swap_color_bytes = 1 }
    };

    if (!jpeg_cfg.outbuf) {
        ESP_LOGE(TAG, "Memory allocation for JPEG output buffer failed");
        return;
    }

    esp_jpeg_image_output_t outimg;
    esp_err_t decode_res = esp_jpeg_decode(&jpeg_cfg, &outimg);

    if (decode_res == ESP_OK) {
        convertToGreyscale((uint16_t*)jpeg_cfg.outbuf, jpeg_cfg.outbuf, SCALED_IMAGE_WIDTH, SCALED_IMAGE_HEIGHT);

        if (is_first_frame) {
            memcpy(reference_frame_buf, jpeg_cfg.outbuf, sizeof(reference_frame_buf));
            is_first_frame = false;
        } else {
            bool motion_detected = false;
            for (int y = (SCALED_IMAGE_HEIGHT - 15) / 2; y < ((SCALED_IMAGE_HEIGHT - 15) / 2) + 15; y++) {
                for (int x = 0; x < SCALED_IMAGE_WIDTH; x++) {
                    int index = y * SCALED_IMAGE_WIDTH + x;
                    if (abs((int)jpeg_cfg.outbuf[index] - (int)reference_frame_buf[index]) > MOTION_THRESHOLD) {
                        motion_detected = true;
                        break;
                    }
                }
                if (motion_detected) {
                    break;
                }
            }
            memcpy(reference_frame_buf, jpeg_cfg.outbuf, SCALED_IMAGE_WIDTH * SCALED_IMAGE_HEIGHT);

            if (motion_detected) {
                ESP_LOGE(TAG, "Motion detected");
            }
        }
    } else {
        ESP_LOGE(TAG, "JPEG Decoding Failed");
    }

    free(jpeg_cfg.outbuf); // Free the allocated memory
    vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));
}*/

camera_fb_t* capture_image(void) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return NULL;
    }
    return fb;
}


void app_main() {
    // Initialize the camera
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    while (1) {
        // Capture an image
        camera_fb_t *fb = capture_image();
        if (fb) {
            ESP_LOGI(TAG, "First two bytes: 0x%02X 0x%02X", fb->buf[0], fb->buf[1]);
            ESP_LOGI(TAG, "Last two bytes: 0x%02X 0x%02X", fb->buf[fb->len - 2], fb->buf[fb->len - 1]); // Process the captured image
            // ... (image processing logic)

            // Return the frame buffer to the camera driver
            esp_camera_fb_return(fb);
        }

        // Delay or other logic as required
    }
}
