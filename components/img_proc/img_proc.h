#ifndef IMG_PROC_H
#define IMG_PROC_H

#include <stdint.h>


void convertToGreyscale(uint16_t* image, uint8_t *greyImage, int width, int height);

void sumPixelsInROI(uint8_t* image, int width, int height, int startX, int startY, int roiWidth, int roiHeight, uint32_t *pixelSum);

void applyGaussianBlurToROI(uint8_t* current_img_buf, uint8_t* temp_blurred_buf);

#endif // IMG_PROC_H
