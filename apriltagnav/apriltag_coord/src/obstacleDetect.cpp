#include "obstacleDetect.h"


ObstacleDetect::ObstacleDetect(/* args */): in_cloud(new PcXYZ()), out_cloud(new PcXYZ()){

    cloudSub = nh.subscribe<PcROS>("/cloud_merge", 1, &ObstacleDetect::cloudCallback, this);
    obstaclePub = nh.advertise<PcROS>("/aprilNav/obstacle_pub_path", 1);

    laserObstacleFlag = false;
    rgbdLaserObstacleFlag = false;
    thDist = MAX_X;
    boxFilter.setMin(Eigen::Vector4f(MIN_X, MIN_Y, MIN_Z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(MAX_X, MAX_Y, MAX_Z, 1.0));
    boxFilter.setInputCloud(in_cloud);

}

ObstacleDetect::~ObstacleDetect(){
}

void ObstacleDetect::cloudCallback(const PcROS_Ptr& cloud_msg){
    
	pcl::fromROSMsg(*cloud_msg, *in_cloud);
    
    boxFilter.setMax(Eigen::Vector4f(thDist, MAX_Y, MAX_Z, 1.0));
    boxFilter.filter(*out_cloud);
    if(out_cloud->size() > 10) laserObstacleFlag = true;
    else laserObstacleFlag = false;
    publishCloud(cloud_msg);
}

void ObstacleDetect::publishCloud(const PcROS_Ptr& cloud_msg){
    PcROS tempCloud;
    pcl::toROSMsg(*out_cloud, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = cloud_msg->header.frame_id;
    if (obstaclePub.getNumSubscribers() != 0)
        obstaclePub.publish(tempCloud);
}

void ObstacleDetect::setThresholdDist(float dist){
    thDist = dist;
}

bool ObstacleDetect::shouldStop(){
    return laserObstacleFlag;
}
