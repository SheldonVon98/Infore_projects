#ifndef OBSTACLE_DETECT_H
#define OBSTACLE_DETECT_H

#define MIN_X 0.0
#define MIN_Y -0.4
#define MIN_Z -0.1

#define MAX_X 2.0
#define MAX_Y 0.4
#define MAX_Z 0.1

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

typedef sensor_msgs::PointCloud2 PcROS;
typedef sensor_msgs::PointCloud2ConstPtr PcROS_Ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PcXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PcXYZ_Ptr;


class ObstacleDetect{
private:
    ros::NodeHandle nh;
    ros::Subscriber cloudSub;

    ros::Publisher obstaclePub;
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    PcXYZ_Ptr in_cloud, out_cloud;
    bool laserObstacleFlag, rgbdLaserObstacleFlag;
    float thDist;

public:
    ObstacleDetect(/* args */);
    ~ObstacleDetect();
    void cloudCallback(const PcROS_Ptr& cloud_msg);
    void publishCloud(const PcROS_Ptr& cloud_msg);
    void setThresholdDist(float dist);
    bool shouldStop();
};




#endif