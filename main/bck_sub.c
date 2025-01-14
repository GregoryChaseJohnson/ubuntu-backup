#include "esp_camera.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "esp_task_wdt.h"
#include <esp_system.h>
#include "img_proc.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>
#include "jpeg_decoder.h"

static const char *TAG = "esp32";

#define CONFIG_XCLK_FREQ 20000000
#define MAX_JPEG_BUFFER_SIZE 60000

#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120

#define RGB565_BYTES_PER_PIXEL 2 // 2 bytes per pixel for RGB565 format

// ROI
#define ROI_START_Y 45
#define ROI_END_Y 75
#define ROI_START_X 0
#define ROI_END_X 159

#define ROI_WIDTH (ROI_END_X - ROI_START_X + 1)
#define ROI_HEIGHT (ROI_END_Y - ROI_START_Y + 1)
#define MOTION_MASK_SIZE (ROI_WIDTH * ROI_HEIGHT)

#define MOTION_THRESHOLD 205

#define BACKGROUND_UPDATE_INTERVAL 30
 
static int frame_counter = 0;

static bool is_first_frame = true; // Initialize the flag

const int background_build_threshold = 25; // 5 initial frames + 20 for building background


static uint8_t out_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT * RGB565_BYTES_PER_PIXEL];
static uint8_t current_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT];
uint8_t reference_frame_buf[ROI_WIDTH * ROI_HEIGHT];

static int initial_frame_count = 0;
const int initial_frame_threshold = 5; // Number of initial frames to ignore

static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {
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
        .frame_size = FRAMESIZE_QQVGA,

        .jpeg_quality = 60,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY};

    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void updateBackgroundIfNeeded(uint8_t *current_blurred_img, uint8_t *reference_img) {
    if (++frame_counter >= BACKGROUND_UPDATE_INTERVAL) {
        for (int y = ROI_START_Y; y <= ROI_END_Y; y++) {
            for (int x = ROI_START_X; x <= ROI_END_X; x++) {
                int roi_idx = (y - ROI_START_Y) * ROI_WIDTH + (x - ROI_START_X);
                int ref_idx = y * IMAGE_WIDTH + x;
                reference_img[ref_idx] = current_blurred_img[roi_idx];
            }
        }
        frame_counter = 0; // Reset the counter
    }
}

void backgroundSubtraction(uint8_t *blurred_current_img, uint8_t *blurred_reference_img, uint8_t *mask) {
    for (int y = 0; y < ROI_HEIGHT; y++) {
        for (int x = 0; x < ROI_WIDTH; x++) {
            int idx = y * ROI_WIDTH + x;
            uint8_t pixel_current = blurred_current_img[idx];
            uint8_t pixel_reference = blurred_reference_img[(ROI_START_Y + y) * IMAGE_WIDTH + (ROI_START_X + x)];
            mask[idx] = (abs(pixel_current - pixel_reference) > MOTION_THRESHOLD) ? 255 : 0;
        }
    }
}

bool SignificantMotion(uint8_t *mask, int size, int motion_threshold) {
    int motion_count = 0;
    for (int i = 0; i < size; i++) {
        if (mask[i] == 255) {
            motion_count++;
        }
    }
    return motion_count > motion_threshold;
}

void updateBackgroundModel(uint8_t *current_img, uint8_t *background_img, int frame_count) {
    for (int i = 0; i < ROI_WIDTH * ROI_HEIGHT; i++) {
        background_img[i] = ((background_img[i] * (frame_count - 1)) + current_img[i]) / frame_count;
    }
}


void convertToGreyscale(uint16_t *input_buf, uint8_t *output_buf, int width, int height);

void image_processing_task(void* pvParameters) {
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = fb->buf,
            .indata_size = fb->len,
            .outbuf = out_img_buf,
            .outbuf_size = sizeof(out_img_buf),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_0,
            .flags = {.swap_color_bytes = 1}
        };

        esp_jpeg_image_output_t outimg;  // Declare the outimg variable here
        esp_err_t decode_res = esp_jpeg_decode(&jpeg_cfg, &outimg);
        
        if (decode_res == ESP_OK) {
            convertToGreyscale((uint16_t *)out_img_buf, current_img_buf, IMAGE_WIDTH, IMAGE_HEIGHT);
            uint8_t temp_blurred_buf[ROI_WIDTH * ROI_HEIGHT];
            applyGaussianBlurToROI(current_img_buf, temp_blurred_buf);

            if (initial_frame_count < initial_frame_threshold) {
                // Skip initial frames for camera stabilization
                initial_frame_count++;
                ESP_LOGI(TAG, "Skipping initial frame %d", initial_frame_count);
            } else if (initial_frame_count < background_build_threshold) {
                // Build background model using averaging
                if (initial_frame_count == initial_frame_threshold) {
                    // Initialize background with the first frame after skipping
                    memcpy(reference_frame_buf, temp_blurred_buf, ROI_WIDTH * ROI_HEIGHT);
                } else {
                    // Incrementally update the background model
                    updateBackgroundModel(temp_blurred_buf, reference_frame_buf, initial_frame_count - initial_frame_threshold + 1);
                }
                initial_frame_count++;
            } else {
                // Regular motion detection and processing
                updateBackgroundIfNeeded(temp_blurred_buf, reference_frame_buf);

                uint8_t motion_mask[MOTION_MASK_SIZE] = {0};
                backgroundSubtraction(temp_blurred_buf, reference_frame_buf, motion_mask);

                if (SignificantMotion(motion_mask, MOTION_MASK_SIZE, MOTION_THRESHOLD)) {
                    ESP_LOGI(TAG, "Significant motion detected.");
                    // Further analysis
                } else {
                    ESP_LOGI(TAG, "*");
                }
            }
        } else {
            ESP_LOGE(TAG, "JPEG Decoding Failed");
        }
        esp_camera_fb_return(fb);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
        
void app_main()
{
    // Initialize the camera
    if (init_camera() != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    xTaskCreate(image_processing_task, "ImageProcessingTask", 16384, NULL, 5, NULL);
}
