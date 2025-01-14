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

uint8_t reference_frame_buf[19200];  

static const char* TAG = "esp32";

#define CONFIG_XCLK_FREQ 20000000 
#define MAX_JPEG_BUFFER_SIZE 60000

#define IMAGE_WIDTH 160  // Width of the image
#define IMAGE_HEIGHT 120 // Height of the image
 
#define RGB565_BYTES_PER_PIXEL 2 // 2 bytes per pixel for RGB565 format

#define ROI_START_Y 45
#define ROI_END_Y 75
#define ROI_WIDTH IMAGE_WIDTH
#define ROI_HEIGHT (ROI_END_Y - ROI_START_Y)

#define SPARSE_FACTOR 2  // Check every 2nd pixel for changes

static bool is_first_frame = true;  // Initialize the flag

static uint8_t out_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT * RGB565_BYTES_PER_PIXEL];
static uint8_t grayscale_img_buf[IMAGE_WIDTH * IMAGE_HEIGHT];

static int consecutive_motion_frames = 0;
const int MAX_CONSECUTIVE_MOTION_FRAMES = 20;

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
        .frame_size = FRAMESIZE_QQVGA,

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

typedef struct {
    int sum;        // Sum of pixel differences in the column
    bool changed;   // Flag indicating if the column has significant change
} ColumnChange;

bool detect_motion_in_columns(ColumnChange* column_changes, int num_columns, int threshold, int min_contiguous_columns) {
    int contiguous_changed_columns = 0;

    for (int i = 0; i < num_columns; i++) {
        if (column_changes[i].sum > threshold) {
            contiguous_changed_columns++;

            if (contiguous_changed_columns >= min_contiguous_columns) {
                return true;  // Motion detected
            }
        } else {
            contiguous_changed_columns = 0;  // Reset count if a column does not meet the threshold
        }
    }

    return false;  // No motion detected
}

typedef struct {
    int sum;        // Sum of pixel differences in the strip
    bool changed;   // Flag indicating if the strip has significant change
} StripChange;

bool detect_motion_in_strips(StripChange* strip_changes, int num_strips, int threshold, int min_contiguous_strips) {
    int contiguous_changed_strips = 0;

    for (int i = 0; i < num_strips; i++) {
        if (strip_changes[i].sum > threshold) {
            contiguous_changed_strips++;

            if (contiguous_changed_strips >= min_contiguous_strips) {
                return true;  // Motion detected
            }
        } else {
            contiguous_changed_strips = 0;  // Reset count if a strip does not meet the threshold
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
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = { .swap_color_bytes = 1 }
    };

    if (!jpeg_cfg.outbuf) {
        ESP_LOGE(TAG, "Memory allocation for JPEG output buffer failed");
        return;
    } 

    esp_jpeg_image_output_t outimg;
    esp_err_t decode_res = esp_jpeg_decode(&jpeg_cfg, &outimg);

    if (decode_res == ESP_OK) {
        convertToGreyscale((uint16_t*)out_img_buf, grayscale_img_buf, IMAGE_WIDTH, IMAGE_HEIGHT);
        // After converting to grayscale
      
        if (is_first_frame) {
            memcpy(reference_frame_buf, grayscale_img_buf, sizeof(reference_frame_buf));
            is_first_frame = false;
        } else {

            bool motion_detected = false;

            int change_threshold = 1000;  // Define based on your application
            int min_contiguous = 3;  
            
            // Column detect logic
            int max_changed_columns = ROI_WIDTH / 2;  // Every other pixel in width
            ColumnChange* column_changes = malloc(max_changed_columns * sizeof(ColumnChange));
            if (column_changes == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for column_changes");
                return; // Handle memory allocation failure
            }

            for (int x = 0; x < ROI_WIDTH; x += 2) {  // Every other pixel in width
                int column_change_sum = 0;
                for (int y = ROI_START_Y; y < ROI_END_Y; y++) {
                    int index = y * IMAGE_WIDTH + x;
                    column_change_sum += abs(grayscale_img_buf[index] - reference_frame_buf[index]);
                }
                column_changes[x / 2].sum = column_change_sum;
                column_changes[x / 2].changed = false;

                
            }

            bool motion_detected_columns = detect_motion_in_columns(column_changes, max_changed_columns, change_threshold, min_contiguous);
            if (motion_detected_columns) {
                ESP_LOGE(TAG, "Motion detected in columns");
            }
            
            free(column_changes);

            int change_threshold_strips = 1500;  // Define based on your application for strip changes
            int min_contiguous_strips = 8;  

            // strip chnage logic
            int num_strips = ROI_WIDTH / 5;
            StripChange* strip_changes = malloc(num_strips* sizeof(StripChange));
            if (strip_changes == NULL) {
                ESP_LOGE(TAG, " Failed to allocate memory for strips");
                return; // handle mem allocation failure

            }

            for (int x = 0; x < ROI_WIDTH; x += 5) {
                int strip_change_sum = 0 ;
                for(int strip_x = x; strip_x < x + 5 && strip_x < ROI_WIDTH; strip_x++) {
                    for (int y = 0; y < ROI_HEIGHT; y += 2) {
                        int index = y * IMAGE_WIDTH + strip_x;
                        strip_change_sum += abs(grayscale_img_buf[index] - reference_frame_buf[index]);
                    }
                }
                
                strip_changes[x / 5].sum = strip_change_sum;
                strip_changes[x / 5].changed = false;

            }

            bool motion_detected_strips = detect_motion_in_strips(strip_changes, num_strips, change_threshold_strips, min_contiguous_strips);   
            if (motion_detected_strips) {
                ESP_LOGE(TAG, "Motion detected in strips");
            }
               
            free(strip_changes);

            // Combined motion detection
            motion_detected = motion_detected_columns || motion_detected_strips;   
            
            if (motion_detected) {
                consecutive_motion_frames++;
                // Reset reference frame if motion is detected in too many consecutive frames
                if (consecutive_motion_frames > MAX_CONSECUTIVE_MOTION_FRAMES) {
                    memcpy(reference_frame_buf, grayscale_img_buf, sizeof(reference_frame_buf));
                    consecutive_motion_frames = 0;
                }    
            } else {
                ESP_LOGI(TAG, "*");
                consecutive_motion_frames = 0;
                // Update reference frame using SMA
                for (int x = 0; x < ROI_WIDTH; x += 2) { 
                    for (int y = ROI_START_Y; y < ROI_END_Y; y++) {   
                        int index = y * IMAGE_WIDTH + x;
                        reference_frame_buf[index] = (grayscale_img_buf[index] + reference_frame_buf[index]) / 2;
                    }
                }    
                     
            }
            
        }
    } else {
        ESP_LOGE(TAG, "JPEG Decoding Failed");
    }
}    
    //vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));

    
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

        //vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));
    }   
}
