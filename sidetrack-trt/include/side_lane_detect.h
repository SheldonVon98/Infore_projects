#ifndef SIDE_LANE_DETECT_H
#define SIDE_LANE_DETECT_H

#define DEBUG
#define ALLOW_SHOW_CV
#define DEVICE              0 // GPU id
#define INPUT_H             704
#define INPUT_W             960
#define NORM_MEAN           0.406
#define NORM_STD            0.229
#define REF_POINT_X         210
#define REF_POINT_Y         INPUT_H
#define INPUT_INDEX         0
#define OUTPUT_INDEX        1
#define BATCH_SIZE          1
#define INPUT_SIZE          3*INPUT_H * INPUT_W
#define OUTPUT_SIZE         3*INPUT_H * INPUT_W
#define MIN_CONTOURS        200
#define CONTOURS_MIN_POS    INPUT_H/2
#define CONTOURS_MIN_LENGTH INPUT_H/2

#define CURVE_FIT_POLY_N    2
#define LINE_FIT_POLY_N     1

#define LANE_CURVE_FIT_TOP      50
#define LANE_CURVE_FIT_BOTTOM   550
#define LANE_CURVE_TOP          50
#define LANE_CURVE_BOTTOM       550
#define LANE_LINE_FIT_TOP       300
#define LANE_LINE_FIT_BOTTOM    INPUT_H
#define LANE_LINE_TOP           550
#define LANE_LINE_BOTTOM        INPUT_H
#define LANE_DIST_COEFF         1.685
#define LANE_STATUS_STEEP_THR   0.005
#define LANE_STATUS_SMOOTH_THR  0.0004

#define SIDE_CURVE_FIT_TOP      50
#define SIDE_CURVE_FIT_BOTTOM   600
#define SIDE_CURVE_TOP          50
#define SIDE_CURVE_BOTTOM       600
#define SIDE_LINE_FIT_TOP       300
#define SIDE_LINE_FIT_BOTTOM    INPUT_H
#define SIDE_LINE_TOP           550
#define SIDE_LINE_BOTTOM        INPUT_H
#define SIDE_DIST_COEFF         1.685
#define SIDE_STATUS_STEEP_THR   0.001
#define SIDE_STATUS_SMOOTH_THR  0.0005
#include <fstream>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "cuda_runtime_api.h"
#include "common.h"
#include "logging.h"

static Logger gLogger;
using namespace nvinfer1;

enum TrackStatus{
    UNKNOWN             = 0, // Known status.
    STRAIGHT            = 1, // straght road.
    SMOOTH_RIGHT_TURN   = 2, // Smooth right turn, can turn.
    STEEP_RIGHT_TURN    = 3, // steep right turn, can not turn.
    SMOOTH_LEFT_TURN    = 4, // Smooth left turn, can turn.
    STEEP_LEFT_TURN     = 5, // steep left turn, can not turn.
};

class SLDetect{
private:
    std::string engine_name = "/home/nvidia/camera_front_side/sideTrack/src/front_trt/fscnn.engine";
    float input_data[INPUT_SIZE];
    float output_prob[OUTPUT_SIZE];
    void *buffers[2];
    cudaStream_t stream;
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    cv::Mat image, img_f32;
    const cv::Point refPoint = cv::Point2i(REF_POINT_X, REF_POINT_Y);
    ros::Publisher frontMsgPub;
    side_track::front_msgs resMsg;
    bool show_cv = false;
#ifdef DEBUG
    image_transport::Publisher imgResPub;
    sensor_msgs::ImagePtr imgMsg;
#endif

private:
#ifdef DEBUG
    const char *TrackStatusHint[6] = {
        "Unknown", "Straight", 
        "Smooth turn ->", "Steep turn ->",
        "Smooth turn <-", "Steep turn <-",
    };
#endif
    TrackStatus trackStatus_side = UNKNOWN;
    TrackStatus trackStatus_lane = UNKNOWN;
    int distance_side = 0;
    int distance_lane = 0;

private:
    void inference();
    /*
    *   Convert cv::Mat to data buffer.
    */
    void mat2buffer(cv::Mat* image);
    /*
    *   Directly convert raw data from 
    *   ros message to data buffer.
    */
    void msgData2buffer(const std::vector<unsigned char> data);
    std::vector<cv::Point2f> getContourCurve(cv::Mat singleClassImg);
    cv::Mat curveFit(std::vector<cv::Point2f> curve, 
                    const int curveFitTop,
                    const int curveFitBottom,
                    const int curveTop,
                    const int curveBottom,
                    const int polyNum);
    void detectStrip(cv::Mat image, 
                    const int curveFitTop,
                    const int curveFitBottom,
                    const int curveTop,
                    const int curveBottom,
                    const double steep_threshold,
                    const double smooth_threshold,
                    TrackStatus *trackStatus,
                    const int lineFitTop,
                    const int lineFitBottom,
                    const int lineTop,
                    const int lineBottom,
                    int *distance,
                    float distance_coeff);
    void detectLane(cv::Mat laneImg);
    void detectSide(cv::Mat sideImg);
    void resProcess();
    cv::Mat* imagePreprocess(cv::Mat img, 
                        int width, 
                        int height);
    void publishMsg();
    void publishImg();
public:
    SLDetect();
    ~SLDetect();
    void setMsgPublisher(ros::Publisher pub);
    void setImgPublisher(image_transport::Publisher pub);
    void rosCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif