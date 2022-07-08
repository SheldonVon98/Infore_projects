#ifndef INFERENCE_H
#define INFERENCE_H

#include "macros.h"
#include "yololayer.h"
#include "NvInfer.h"
#include "cuda_utils.h"

using namespace std;
using namespace nvinfer1;

float iou(float lbox[4], 
        float rbox[4]);
bool cmp(const Yolo::Detection& a, 
        const Yolo::Detection& b);
void nms(std::vector<Yolo::Detection>& res, 
        float *output, 
        float conf_thresh, 
        float nms_thresh = 0.5);
cv::Rect get_rect(cv::Mat& img, 
                float bbox[4]);
void doInference(IExecutionContext& context, 
                            cudaStream_t& stream, 
                            void **buffers, 
                            float* input, 
                            int inputSize,
                            float* output,
                            int outputSize);
#endif