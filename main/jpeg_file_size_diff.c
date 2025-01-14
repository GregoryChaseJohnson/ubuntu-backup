#include <esp_system.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "connect_wifi.h"
#include "esp_task_wdt.h"


static const char *TAG = "esp32-cam Webserver";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static bool wifi_failed = false;



#define CONFIG_XCLK_FREQ 20000000 
#define DESIRED_FPS 7
#define DESIRED_FRAME_TIME_MS (100/ DESIRED_FPS)
#define FILE_SIZE_BUFFER_LENGTH 14
#define FILE_SIZE_CHANGE_THRESHOLD 500 // Adjust this value HIGHER to reduce false postives 

#define TWDT_TIMEOUT_S 5  // Time in seconds before the watchdog timer triggers a timeout
#define TASK_RESET_PERIOD_MS 2000  // The period in milliseconds to reset the TWDT

static size_t file_size_buffer[FILE_SIZE_BUFFER_LENGTH] = {0};
static int buffer_index = 0;

void send_detection_notification() {
    ESP_LOGI(TAG, "Motion detected!");
}

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

        .jpeg_quality = 20,
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

void update_file_size_buffer(size_t new_size) {
    file_size_buffer[buffer_index] = new_size;
    buffer_index = (buffer_index + 1) % FILE_SIZE_BUFFER_LENGTH;
}

size_t calculate_average_file_size() {
    size_t sum = 0;
    for (int i = 0; i < FILE_SIZE_BUFFER_LENGTH; i++) {
        sum += file_size_buffer[i];
    }
    return sum / FILE_SIZE_BUFFER_LENGTH;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t *_jpg_buf;
    char part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        int64_t frame_start_time = esp_timer_get_time();
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if (!jpeg_converted) {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        // Motion detection logic should be placed here
        size_t current_jpg_size = _jpg_buf_len; // Size of the current JPEG image
        update_file_size_buffer(current_jpg_size);

        size_t average_jpg_size = calculate_average_file_size();
        if (abs((int)current_jpg_size - (int)average_jpg_size) > FILE_SIZE_CHANGE_THRESHOLD) {
            ESP_LOGI(TAG, "Significant change in image size detected, possible motion");
            send_detection_notification();
        }

        // Retry logic with exponential backoff
        int max_retries = 5;
        int retry_delay = 100;
        for (int retry_count = 0; retry_count < max_retries && res == ESP_OK; ++retry_count) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            if (res == ESP_OK) {
                size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
                res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            }
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            }

            if (res != ESP_OK) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    ESP_LOGW(TAG, "Send failed, retrying in %d ms", retry_delay);
                    vTaskDelay(pdMS_TO_TICKS(retry_delay));
                    retry_delay *= 2;
                    res = ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Send failed with error %d", errno);
                    break;
                }
            } else {
                break;
            }
        }

        if (fb->format != PIXFORMAT_JPEG) {
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - frame_start_time;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %luKB %lums (%.1ffps)",
                 (uint32_t)(_jpg_buf_len / 1024), 
                 (uint32_t)frame_time, 
                 1000.0 / (uint32_t)frame_time);

        int64_t delay_time = DESIRED_FRAME_TIME_MS - frame_time;
        if (delay_time > 0) {
            vTaskDelay(delay_time / portTICK_PERIOD_MS);
        }
    }

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    return res;
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL
};

httpd_handle_t setup_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd = NULL;

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &uri_get);
    }

    return stream_httpd;
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    connect_wifi();

    if (wifi_connect_status) {
        esp_err_t err = init_camera();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera Init Failed");
            return;
        }
        setup_server();
        ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running");
    } else {
        ESP_LOGI(TAG, "Failed to connect to Wi-Fi");
        wifi_failed = true; // set flag
    }

        while (true) {
            if (!wifi_failed) {
                ESP_ERROR_CHECK(esp_task_wdt_reset());  // Reset TWDT if Wi-Fi didn't fail
            }
            vTaskDelay(pdMS_TO_TICKS(1000));  // Delay to prevent tight loop
        }
}


