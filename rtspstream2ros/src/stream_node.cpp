#include "retrieve.h"

#define DIST_PARAM_SIZE   5
#define INTRIN_MAT_SIZE   9
image_transport::Publisher imgRawPub, imgCalPub;
ros::Publisher camInfoPub;
std_msgs::Header header;
cv::Mat mapFrame, map1, map2;
sensor_msgs::ImagePtr rawImgMsg, imgMsg;
sensor_msgs::CameraInfo cam_info;
bool show_cv;

int main(int argc, char** argv) {

    header.frame_id = "CamCalImage";
    ros::init(argc, argv, "rtsp_stream_node");
    ros::NodeHandle n;

    std::string ImageRawNode, ImageCalNode, CamInfo, rtspURL, CamName, CamParam;
    ros::param::get(ros::this_node::getName()+"/rtspURL", rtspURL);
    ros::param::get(ros::this_node::getName()+"/ImageRawNode", ImageRawNode);
    ros::param::get(ros::this_node::getName()+"/ImageCalNode", ImageCalNode);
    ros::param::get(ros::this_node::getName()+"/CamInfoNode", CamInfo);
    ros::param::get(ros::this_node::getName()+"/show_cv", show_cv);
    ros::param::get(ros::this_node::getName()+"/CamName", CamName);
    ros::param::get(ros::this_node::getName()+"/CamParam", CamParam);

    cam_info = camera_info_manager::CameraInfoManager(n, CamName, CamParam).getCameraInfo();
    double distValue[DIST_PARAM_SIZE], IntrinsicValue[INTRIN_MAT_SIZE];
    for(int i=0; i<DIST_PARAM_SIZE; i++) distValue[i] = cam_info.D[i];
    for(int i=0; i<INTRIN_MAT_SIZE; i++) IntrinsicValue[i] = cam_info.K[i];
    cv::Mat IntrinsicMat = cv::Mat(3, 3, CV_64F, IntrinsicValue);
    cv::Mat distMat = cv::Mat(1, 5, CV_64F, distValue);
    cv::initUndistortRectifyMap(IntrinsicMat, 
                                distMat, 
                                cv::Mat(), 
                                cv::Mat(), 
                                cv::Size(cam_info.width, cam_info.height), 
                                CV_32FC1, 
                                map1, map2);

    image_transport::ImageTransport it(n);

    imgRawPub = it.advertise(ImageRawNode, 1);
    imgCalPub = it.advertise(ImageCalNode, 1);

    camInfoPub = n.advertise<sensor_msgs::CameraInfo>(CamInfo, 1000);
    Retrieve *retrieve = new Retrieve(rtspURL);

    retrieve->run([](AVCodecContext* codec_ctx, AVFrame* picture_rgb) {
        cv::Mat mat(codec_ctx->height, 
                    codec_ctx->width, 
                    CV_8UC3, 
                    picture_rgb->data[0], 
                    picture_rgb->linesize[0]);
        if (mat.empty()){
            std::cerr << "frame not grabbed\n";
            return;
        } else {
            header.stamp = ros::Time::now();
            cam_info.header = header;

            rawImgMsg = cv_bridge::CvImage(std_msgs::Header(), 
                            "bgr8", 
                            mat).toImageMsg();
            cv::remap(mat, mapFrame, map1, map2, cv::INTER_LINEAR);
            imgMsg = cv_bridge::CvImage(header, 
                            "bgr8", 
                            mapFrame).toImageMsg();
            imgRawPub.publish(rawImgMsg);
            imgCalPub.publish(imgMsg);
            camInfoPub.publish(cam_info);
            if (show_cv){
                cv::imshow("stream", mapFrame);
                cv::waitKey(1);
            }
        }
    });
    ros::spin();
}