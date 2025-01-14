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

uint8_t reference_frame[19200];  // Adjusted size

const int MOTION_THRESHOLD = 150;
const int FRAME_CAPTURE_INTERVAL_MS = 800;
const int CHANGE_THRESHOLD = 1000;
const int PIXEL_THRESHOLD = 500;

//const int CHANGE_THRESHOLD = 550;  // Threshold for initial change detection
//const int PIXEL_THRESHOLD = 100;   // Threshold for similarity check in validate_neighbors


static const char* TAG = "esp32-cam";

#define CONFIG_XCLK_FREQ 20000000 
#define MAX_JPEG_BUFFER_SIZE 60000

#define IMAGE_WIDTH 160  // Width of the image
#define IMAGE_HEIGHT 120 // Height of the image

#define SCALED_IMAGE_WIDTH (IMAGE_WIDTH)  
#define SCALED_IMAGE_HEIGHT (IMAGE_HEIGHT)  
#define RGB565_BYTES_PER_PIXEL 2 // 2 bytes per pixel for RGB565 format

#define ROI_START_Y 45
#define ROI_END_Y 75
#define ROI_WIDTH SCALED_IMAGE_WIDTH
#define ROI_HEIGHT (ROI_END_Y - ROI_START_Y)

#define SPARSE_FACTOR 2  // Check every 2nd pixel for changes
#define FRAME_HISTORY_COUNT 3  // Number of frames to keep in history
#define REFERENCE_FRAME_UPDATE_INTERVAL 30  // Update reference frame every 30 frames
#define PIXEL_COUNT 2400  // Number of pixels in one grayscale image
#define PIXEL_SIZE sizeof(uint16_t)  // Size of each pixel in bytes
#define INVALID_FRAME_VALUE 0xFFFF
#define PIXELS_PER_FRAME (ROI_WIDTH / 2 * ROI_HEIGHT)


static uint16_t frame_history[FRAME_HISTORY_COUNT][PIXELS_PER_FRAME] = {0};
static int current_frame_index = 0;
static int frame_counter = 0;

static bool is_first_frame = true;  // Initialize the flag

static uint8_t out_img_buf[SCALED_IMAGE_WIDTH * SCALED_IMAGE_HEIGHT * RGB565_BYTES_PER_PIXEL];
static uint8_t current_frame[SCALED_IMAGE_WIDTH * SCALED_IMAGE_HEIGHT];

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
        .frame_size = FRAMESIZE_QVGA,

        .jpeg_quality = 60,
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

void initializeFrameHistory() {
    for (int i = 0; i < FRAME_HISTORY_COUNT; i++) {
        // Mark the frame as invalid initially
        memset(frame_history[i], INVALID_FRAME_VALUE, PIXEL_COUNT * PIXEL_SIZE);
    }
    current_frame_index = 0;
}

typedef struct {
    int sum;        // Sum of pixel differences in the column
    bool changed;   // Flag indicating if the column has significant change
} ColumnChange;

void updateFrameHistory(uint16_t* current_frame) {
    // Copy the current frame into the frame history at the current index
    memcpy(frame_history[current_frame_index], current_frame, PIXELS_PER_FRAME * sizeof(uint16_t));

    // Update the current frame index for the rolling buffer
    current_frame_index = (current_frame_index + 1) % FRAME_HISTORY_COUNT;
}

bool validate_neighbors(uint16_t** frame_history, int flagged_column, int num_columns, int num_frames, int pixel_threshold) {
    for (int frame = 1; frame < num_frames; frame++) {        
        for (int offset = -1; offset <= 1; offset += 2) { // Check both left and right neighbors
            int neighbor_column = flagged_column + offset;
            if (neighbor_column < 0 || neighbor_column >= num_columns) continue;

            for (int y = ROI_START_Y; y < ROI_END_Y; y++) {
                int current_pixel_index = y * SCALED_IMAGE_WIDTH + flagged_column;
                int neighbor_pixel_index = y * SCALED_IMAGE_WIDTH + neighbor_column;

                if (frame_history[frame][neighbor_pixel_index] == INVALID_FRAME_VALUE) {
                    continue;
                }

                int pixel_difference = abs(frame_history[0][current_pixel_index] - frame_history[frame][neighbor_pixel_index]);
                if (pixel_difference > pixel_threshold) {
                    return false; // Pixel difference too large, indicating dissimilarity
                }
            }
        }
    }
    return true; // Neighboring columns in future frames have similar pixel patterns
}

bool detect_motion_in_columns(ColumnChange* column_changes, uint16_t** frame_history, int num_columns, int num_frames, int threshold, int pixel_threshold) {
    for (int i = 0; i < num_columns; i++) {
        if (column_changes[i].sum > threshold) {
            if (validate_neighbors(frame_history, i, num_columns, num_frames, pixel_threshold)) {
                return true;  // Motion detected in this column
            }
        }
    }

    return false;  // No motion detected
}


void convertToGreyscale(uint16_t* input_buf, uint8_t* output_buf, int width, int height);

void image_processing_task(uint8_t* jpg_buf, size_t jpg_buf_len) {
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = jpg_buf,
        .indata_size = jpg_buf_len,
        .outbuf = out_img_buf,
        .outbuf_size = sizeof(out_img_buf),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_1_2,
        .flags = { .swap_color_bytes = 1 }
    };

    if (!jpeg_cfg.outbuf) {
        ESP_LOGE(TAG, "Memory allocation for JPEG output buffer failed");
        return;
    }

    esp_jpeg_image_output_t outimg;
    esp_err_t decode_res = esp_jpeg_decode(&jpeg_cfg, &outimg);

    if (decode_res == ESP_OK) {
        convertToGreyscale((uint16_t*)out_img_buf, current_frame, SCALED_IMAGE_WIDTH, SCALED_IMAGE_HEIGHT);
     
        if (is_first_frame) {
            memcpy(reference_frame, current_frame, sizeof(reference_frame));
            initializeFrameHistory((uint16_t*)current_frame);
            is_first_frame = false;
        } else {
            updateFrameHistory((uint16_t*)current_frame);
            frame_counter++;

            // Allocate and calculate column changes
            int max_changed_columns = ROI_WIDTH / 2;  // Every other pixel in width
            ColumnChange* column_changes = malloc(max_changed_columns * sizeof(ColumnChange));
            if (column_changes == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for column_changes");
                return; // Handle memory allocation failure
            }

            for (int x = 0; x < ROI_WIDTH; x += 2) {  // Every other pixel in width
                int column_change_sum = 0;
                for (int y = ROI_START_Y; y < ROI_END_Y; y++) {
                    int index = y * SCALED_IMAGE_WIDTH + x;
                    column_change_sum += abs(current_frame[index] - reference_frame[index]);
                }
                column_changes[x / 2].sum = column_change_sum;
            }

            int num_columns = max_changed_columns;
            int num_frames = FRAME_HISTORY_COUNT;
            bool motion_detected = detect_motion_in_columns(column_changes, (uint16_t**)frame_history, num_columns, num_frames, CHANGE_THRESHOLD, PIXEL_THRESHOLD);

            free(column_changes); // Free the dynamically allocated memory for column_changes

            if (motion_detected) {
                ESP_LOGE(TAG, "Motion detected in columns");
            } else {
                ESP_LOGI(TAG, "No motion detected");
            }

            if (frame_counter % REFERENCE_FRAME_UPDATE_INTERVAL == 0) {   
             memcpy(reference_frame, current_frame, sizeof(reference_frame));
            }
        }
    } else {
        ESP_LOGE(TAG, "JPEG Decoding Failed");
    }

    //vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));
}
    
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
           image_processing_task(fb->buf, fb->len);

          // Return the frame buffer to the camera driver
            esp_camera_fb_return(fb);
        }

        vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));
    }   
}
