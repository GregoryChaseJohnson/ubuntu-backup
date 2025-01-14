#include "img_proc.h"

#define ROI_START_Y 45
#define ROI_END_Y 75
#define ROI_START_X 0
#define ROI_END_X 159

#define ROI_WIDTH (ROI_END_X - ROI_START_X + 1)
#define ROI_HEIGHT (ROI_END_Y - ROI_START_Y + 1)

// Function to convert a single RGB565 pixel to grayscale
static uint8_t convertRGB565toGreyscale(uint16_t rgb565Pixel) {
    // Extracting RGB components from a 16-bit RGB565 format
    uint8_t r = (rgb565Pixel >> 11) & 0x1F;
    uint8_t g = (rgb565Pixel >> 5) & 0x3F;
    uint8_t b = rgb565Pixel & 0x1F;

    // Scaling values from their bit-limited range to [0, 255]
    uint8_t red = (r * 255) / 31;
    uint8_t green = (g * 255) / 63;
    uint8_t blue = (b * 255) / 31;

    // Applying weighted average formula for greyscale
    return (uint8_t)(0.3 * red + 0.59 * green + 0.11 * blue);
}

// Function to convert an entire image to grayscale
void convertToGreyscale(uint16_t *image, uint8_t *greyImage, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        greyImage[i] = convertRGB565toGreyscale(image[i]);
    }
}

// Declare the gaussian_kernel as a static constant outside the function
static const float gaussian_kernel[3][3] = {
    {1.0 / 16, 2.0 / 16, 1.0 / 16},
    {2.0 / 16, 4.0 / 16, 2.0 / 16},
    {1.0 / 16, 2.0 / 16, 1.0 / 16}
};

int clamp(int value, int min, int max); // Function prototype

void applyGaussianBlurToROI(uint8_t* current_img_buf, uint8_t* temp_blurred_buf) {
    for (int y = 0; y < ROI_HEIGHT; y++) {
        for (int x = 0; x < ROI_WIDTH; x++) {
            float blurred_pixel = 0.0;

            for (int ky = -1; ky <= 1; ky++) {
                int pixel_y = clamp(y + ky, 0, ROI_HEIGHT - 1);
                int y_index = pixel_y * ROI_WIDTH;

                for (int kx = -1; kx <= 1; kx++) {
                    int pixel_x = clamp(x + kx, 0, ROI_WIDTH - 1);
                    blurred_pixel += current_img_buf[y_index + pixel_x] * gaussian_kernel[ky + 1][kx + 1];
                }
            }

            temp_blurred_buf[y * ROI_WIDTH + x] = (uint8_t)blurred_pixel;
        }
    }
}

// Helper function to clamp values
int clamp(int value, int min, int max) {
    return value < min ? min : (value > max ? max : value);
}
