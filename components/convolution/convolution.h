
#ifndef CONVOLUTION_H
#define CONVOLUTION_H

void dsps_conv_f32_ae32(const float *Signal, int siglen, const float *Kernel, int kernlen, float *convout);

#endif // CONVOLUTION_H
