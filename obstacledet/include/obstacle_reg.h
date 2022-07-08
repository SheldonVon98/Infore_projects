#ifndef OBSTACLE_REGRESSION_H
#define OBSTACLE_REGRESSION_H

#include "common.h"
#include <opencv2/opencv.hpp>

// Coordinate of Original point
#define ORIG_X 150
#define ORIG_Y 300
#define ORIGINAL ORIG_X, ORIG_Y

// Size of the laser image 
#define LASER_IMAGE_H   400
#define LASER_IMAGE_W   400
#define LASER_IMAGE_SIZE LASER_IMAGE_H, LASER_IMAGE_W

class ObstacleReg
{
private:
    ros::NodeHandle nh;
    ros::Subscriber cloudSub;
    ros::Publisher obstacles_pub;
    cv::Point ori;

public:
    ObstacleReg(/* args */);
    ~ObstacleReg();
    void cloudCallback(const PcROS_Ptr& cloud_msg);
};

#endif