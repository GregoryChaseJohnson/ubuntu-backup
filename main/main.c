#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "esp_dsp.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "jpeg_decoder.h"
#include "img_proc.h"
#include "camera_pins.h"

#define CONFIG_XCLK_FREQ 10000000  // Adjusted clock frequency to slow down frame rate
#define IMAGE_HEIGHT 120
#define IMAGE_WIDTH 160
#define RGB565_BYTES_PER_PIXEL 2

// Define the region of interest (ROI)
#define ROI_X 48
#define ROI_Y 68
#define ROI_WIDTH 21  // Reduced ROI width
#define ROI_HEIGHT 21 // Reduced ROI height
#define PADDING 1     // Padding for convolution

static const char *TAG = "camera_sobel";

// Statically allocate buffers
static uint8_t grayscale_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT];
static float grayscale_float_buf[(ROI_WIDTH + 2 * PADDING) * (ROI_HEIGHT + 2 * PADDING)];
static uint8_t out_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT * RGB565_BYTES_PER_PIXEL];

// Camera Initialization Function
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
        .xclk_freq_hz = CONFIG_XCLK_FREQ,  // Reduced clock frequency
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QQVGA,
        .jpeg_quality = 10,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

// Grayscale Conversion Function
void convertToGreyscale(uint16_t *image, uint8_t *greyImage, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        uint16_t pixel = image[i];
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        uint8_t red = (r * 255) / 31;
        uint8_t green = (g * 255) / 63;
        uint8_t blue = (b * 255) / 31;
        greyImage[i] = (uint8_t)(0.3 * red + 0.59 * green + 0.11 * blue);
    }
}

// Convert uint8_t grayscale image to float
void convertToFloat(uint8_t *input, float *output, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        output[i] = (float)input[i];
    }
}

// Add padding to the grayscale image
void add_padding(float *input, float *output, int width, int height, int padding) {
    int padded_width = width + 2 * padding;
    int padded_height = height + 2 * padding;
    
    // Initialize the output buffer to zero (padding area)
    memset(output, 0, padded_width * padded_height * sizeof(float));

    // Copy the input image to the center of the output buffer
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            output[(y + padding) * padded_width + (x + padding)] = input[y * width + x];
        }
    }
}

// Convolution Function using ESP-DSP
void conv2d_with_dsps(float *input, int width, int height, float *kernel, int kernel_size, float *output) {
    int output_width = width - kernel_size + 1;
    static float row_output[1024]; // Static buffer based on the maximum width

    for (int i = 0; i < height - kernel_size + 1; i++) {
        // Perform convolution on each row
        dsps_conv_f32_ae32(&input[i * width], width, kernel, kernel_size, row_output);
        // Copy the row output to the final output buffer
        for (int j = 0; j < output_width; j++) {
            output[i * output_width + j] = row_output[j];
        }
    }
}

void edge_detection_task(void *pvParameters) {
    float kernel_h[9] = { -1, -2, -1, 0, 0, 0, 1, 2, 1 };
    float kernel_v[9] = { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
    int kernel_size = 3;
    int threshold = 150; // Increased threshold for edge detection

    while (true) {
        // Capture a frame from the camera
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // Convert the JPEG image to grayscale
        uint8_t *jpg_buf = fb->buf;
        size_t jpg_buf_len = fb->len;
        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = jpg_buf,
            .indata_size = jpg_buf_len,
            .outbuf = out_img_buf,
            .outbuf_size = IMAGE_WIDTH * IMAGE_HEIGHT * RGB565_BYTES_PER_PIXEL,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .flags = {
                .swap_color_bytes = 1,
            }
        };
        esp_jpeg_image_output_t outimg;

        esp_err_t decode_res = esp_jpeg_decode(&jpeg_cfg, &outimg);
        if (decode_res == ESP_OK) {
            convertToGreyscale((uint16_t*)out_img_buf, grayscale_img_buf, IMAGE_WIDTH, IMAGE_HEIGHT);

            // Extract the region of interest (ROI) and convert to float
            float temp_buf[ROI_WIDTH * ROI_HEIGHT];
            for (int y = 0; y < ROI_HEIGHT; y++) {
                for (int x = 0; x < ROI_WIDTH; x++) {
                    temp_buf[y * ROI_WIDTH + x] = (float)grayscale_img_buf[(ROI_Y + y) * IMAGE_WIDTH + (ROI_X + x)];
                }
            }

            // Add padding to the ROI
            add_padding(temp_buf, grayscale_float_buf, ROI_WIDTH, ROI_HEIGHT, PADDING);

            int padded_width = ROI_WIDTH + 2 * PADDING;
            int padded_height = ROI_HEIGHT + 2 * PADDING;
            int output_width = padded_width - kernel_size + 1;
            int output_height = padded_height - kernel_size + 1;
            float output_h[output_width * output_height];
            float output_v[output_width * output_height];

            conv2d_with_dsps(grayscale_float_buf, padded_width, padded_height, kernel_h, kernel_size, output_h);
            conv2d_with_dsps(grayscale_float_buf, padded_width, padded_height, kernel_v, kernel_size, output_v);

            // Combine the horizontal and vertical edge detection results
            float combined_output[output_width * output_height];
            for (int i = 0; i < output_height; i++) {
                for (int j = 0; j < output_width; j++) {
                    combined_output[i * output_width + j] = 
                        sqrtf(output_h[i * output_width + j] * output_h[i * output_width + j] +
                              output_v[i * output_width + j] * output_v[i * output_width + j]);
                }
            }

            // Confirmation print statement
            printf("Edge detection completed for ROI.\n");
        } else {
            ESP_LOGE(TAG, "JPEG Decoding Failed");
        }

        // Return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);

        // Delay to synchronize with the camera frame rate
        vTaskDelay(400 / portTICK_PERIOD_MS);  // Adjust delay based on processing time and frame rate
    }

    vTaskDelete(NULL);
}

// Main Application Entry Point
void app_main() {
    esp_err_t res = init_camera();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }

    if (xTaskCreate(edge_detection_task, "Edge Detection Task", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
    }
}
