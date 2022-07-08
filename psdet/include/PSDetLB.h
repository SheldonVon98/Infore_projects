#ifndef PSDET_LB_H
#define PSDET_LB_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include "macros.h"
#include "cuda_utils.h"
#include "NvInfer.h"
#include "yololayer.h"
#include "logging.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include "inference.h"
#include "deduction.h"
#include <chrono>

using namespace std;
using namespace nvinfer1;

class PSDetectLB{

private:
    ros::NodeHandle nh;
    ros::Publisher PSDinfo_pub;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher imageNode;
    image_transport::Subscriber sensorNode;
    string rImgName;
    string engine_name;
    int img_w, img_h;
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1

    void* buffers[2];
    ICudaEngine* engine = nullptr;
    cudaStream_t stream;
    IExecutionContext* context = nullptr;

    Deduction deduction;
public:
    // PSDetectLB() = default;
    // ~PSDetectLB() = default;
    PSDetectLB();
    // void onInit();
    void transaction();
    void getLaunchParams();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void inference();
    cv::Mat inference(cv::Mat img, 
                            IExecutionContext& context, 
                            cudaStream_t &stream,void** buffers, 
                            float* data, 
                            float* prob, 
                            std::vector<Yolo::Detection> &result_res);
    void loadEngine();
    ~PSDetectLB();
};

#endif
