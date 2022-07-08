#ifndef OBSTACLE_DETECTION
#define OBSTACLE_DETECTION

#include "common.h"
#include "ground_seg.h"


using namespace std;

class RGBD_Proc {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointSub;
    ros::Publisher obstacle_pub;
    ros::Publisher ground_pub;
    ros::Publisher laser_pub;
    GroundSeg *groundSeg;

    // Container for original & filtered data
    PcXYZ_Ptr cloud;
    PcXYZ_Ptr obstacleCloud;
    PcXYZ_Ptr groundCloud;
    PcXYZ_Ptr laserCloud;

    float leafSize;
    float bottomLimit;
    float topLimit;
    float frontLimit;
    float backLimit;

    string point_sub_path;
    string ground_pub_path;
    string laser_pub_path;

    string obstacle_pub_path;
    string upward_axis_name;
    Eigen::Vector3f upward_axis;
    string forward_axis_name;

public:
    RGBD_Proc();
    ~RGBD_Proc();
    void pc_callback(const PcROS_Ptr& cloud_msg);

private:
    void getParams();
    void filterCloud(PcXYZ_Ptr cloud);
    void publishCloud(const PcROS_Ptr& cloud_msg);
    void clearClouds();
};

#endif