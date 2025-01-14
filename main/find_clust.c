typedef struct {
    int x;
    int y;
} PixelCoord;

void find_clusters(uint8_t* diff_frame, int width, int height) {
    PixelCoord changed_pixels[width * height];  // Array to store changed pixel coordinates
    int changed_pixel_count = 0;

    // Collect coordinates of changed pixels
    for (int y = 0; y < height; y += SPARSE_FACTOR) {
        for (int x = 0; x < width; x += SPARSE_FACTOR) {
            int index = y * width + x;
            if (diff_frame[index]) {
                changed_pixels[changed_pixel_count++] = (PixelCoord){x, y};
            }
        }
    }

    // Logic to find clusters from changed pixels...
    for (int i = 0; i < changed_pixel_count; i++) {
        printf("Changed Pixel: (%d, %d)\n", changed_pixels[i].x, changed_pixels[i].y);
    }
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
        convertToGreyscale((uint16_t*)out_img_buf, grayscale_img_buf, SCALED_IMAGE_WIDTH, SCALED_IMAGE_HEIGHT);
     
        if (is_first_frame) {
            memcpy(reference_frame_buf, grayscale_img_buf, sizeof(reference_frame_buf));
            is_first_frame = false;
    
        } else {
            bool motion_detected = false;
            ESP_LOGI(TAG, "Starting motion detection loop");

            // Dynamically allocate memory for changed_pixels
            int max_changed_pixels = ROI_WIDTH * ROI_HEIGHT / SPARSE_FACTOR / SPARSE_FACTOR;
            PixelCoord* changed_pixels = malloc(max_changed_pixels * sizeof(PixelCoord));
            if (changed_pixels == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for changed_pixels");
                return; // Handle memory allocation failure
            }

            int changed_pixel_count = 0;
            for (int y = ROI_START_Y; y < ROI_END_Y; y += SPARSE_FACTOR) {
                for (int x = 0; x < SCALED_IMAGE_WIDTH; x += SPARSE_FACTOR) {
                    int index = y * SCALED_IMAGE_WIDTH + x;
                    if (abs(grayscale_img_buf[index] - reference_frame_buf[index]) > MOTION_THRESHOLD) {
                        if (changed_pixel_count < max_changed_pixels) {
                        changed_pixels[changed_pixel_count++] = (PixelCoord){x, y};
                    }
                }
            }
        }

            // Print coordinates for now
            for (int i = 0; i < changed_pixel_count; i++) {
                ESP_LOGI(TAG, "Changed Pixel: (%d, %d)", changed_pixels[i].x, changed_pixels[i].y);
            }

            if (changed_pixel_count > 0) {
                motion_detected = true;
                ESP_LOGE(TAG, "Motion detected");
            }   

            // Free the dynamically allocated memory
            free(changed_pixels);

            // Update the reference frame buffer
            memcpy(reference_frame_buf, grayscale_img_buf, sizeof(reference_frame_buf));

            if (motion_detected) {
                ESP_LOGE(TAG, "Motion detected");
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

        //vTaskDelay(pdMS_TO_TICKS(FRAME_CAPTURE_INTERVAL_MS));
    }   
}
