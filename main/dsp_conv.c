#include <stdio.h>
#include <stdlib.h>
#include "esp_dsp.h"

// General 2D convolution function using ESP-DSP optimized 1D convolution
void conv2d_with_dsps(float *input, int width, int height, float *kernel, int kernel_size, float *output) {
    int output_width = width - kernel_size + 1;
    int output_height = height - kernel_size + 1;
    float *row_output = malloc(output_width * sizeof(float));
    if (!row_output) {
        printf("Memory allocation failed\n");
        return;
    }

    for (int i = 0; i < height; i++) {
        dsps_conv_f32_ae32(&input[i * width], width, kernel, kernel_size, row_output);
        for (int j = 0; j < output_width; j++) {
            output[i * output_width + j] = row_output[j];
        }
    }
    free(row_output);
}

void test_sobel_edge_detection() {
    const int width = 5, height = 5, kernel_size = 3;
    float input[25] = { 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 5, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1 };
    float kernel_h[9] = { -1, -2, -1, 0, 0, 0, 1, 2, 1 };
    float kernel_v[9] = { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
    float output_h[(width - kernel_size + 1) * (height - kernel_size + 1)];
    float output_v[(width - kernel_size + 1) * (height - kernel_size + 1)];

    conv2d_with_dsps(input, width, height, kernel_h, kernel_size, output_h);
    conv2d_with_dsps(input, width, height, kernel_v, kernel_size, output_v);

    printf("Output Horizontal Edges: ");
    for (int i = 0; i < sizeof(output_h) / sizeof(output_h[0]); i++) {
        printf("%f ", output_h[i]);
    }
    printf("\nOutput Vertical Edges: ");
    for (int i = 0; i < sizeof(output_v) / sizeof(output_v[0]); i++) {
        printf("%f ", output_v[i]);
    }
    printf("\n");
}

void app_main() {
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    test_sobel_edge_detection();
}
