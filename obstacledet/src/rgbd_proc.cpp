#include "rgbd_proc.h"

RGBD_Proc::RGBD_Proc(): 
    cloud(new PcXYZ),
    groundCloud(new PcXYZ),
    laserCloud(new PcXYZ),
    obstacleCloud(new PcXYZ){
    // Get params from launch file.
    getParams();
    groundSeg = new GroundSeg(upward_axis, 
                            cloud, 
                            groundCloud, 
                            obstacleCloud);
    // Create a ROS subscriber for the input point cloud
    pointSub = nh.subscribe(point_sub_path, 1, 
                            &RGBD_Proc::pc_callback, 
                            this);
    // Create a ROS publisher for the output point cloud
    obstacle_pub = nh.advertise<PcROS>(obstacle_pub_path, 1);
    ground_pub = nh.advertise<PcROS>(ground_pub_path, 1);

    laser_pub = nh.advertise<PcROS>(laser_pub_path, 1);

}

void RGBD_Proc::getParams(){
    ros::param::get(ros::this_node::getName()+"/obstacle_pub", obstacle_pub_path);
    ros::param::get(ros::this_node::getName()+"/ground_pub", ground_pub_path);
    ros::param::get(ros::this_node::getName()+"/laser_pub", laser_pub_path);
    ros::param::get(ros::this_node::getName()+"/point_sub", point_sub_path);
    ros::param::get(ros::this_node::getName()+"/upward_axis", upward_axis_name);
    ros::param::get(ros::this_node::getName()+"/leafSize", leafSize);
    ros::param::get(ros::this_node::getName()+"/topLimit", topLimit);
    ros::param::get(ros::this_node::getName()+"/bottomLimit", bottomLimit);
    
    ros::param::get(ros::this_node::getName()+"/forward_axis", forward_axis_name);
    ros::param::get(ros::this_node::getName()+"/frontLimit", frontLimit);
    ros::param::get(ros::this_node::getName()+"/backLimit", backLimit);

    string ground_normal;
    ros::param::get(ros::this_node::getName()+"/ground_normal", ground_normal);

    
    assert((ground_normal == "x") || 
            (ground_normal == "y") || 
            (ground_normal == "z"));
    if(ground_normal == "x"){
        upward_axis = Eigen::Vector3f(1,0,0);
    } else if(ground_normal == "y"){
        upward_axis = Eigen::Vector3f(0,1,0);
    } else if(ground_normal == "z"){
        upward_axis = Eigen::Vector3f(0,0,1);
    }
}


void RGBD_Proc::pc_callback(const PcROS_Ptr& cloud_msg){
  	// Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *cloud);
    filterCloud(cloud);
    groundSeg->removeGround();
    

    pcl::PointXYZ point;
    for (size_t i = 0; i < obstacleCloud->size(); i++){
        point.x = obstacleCloud->points[i].x;
        point.y = 0;
        point.z = obstacleCloud->points[i].z;
        laserCloud->push_back(point);
    }

    publishCloud(cloud_msg);
    clearClouds();
}

void RGBD_Proc::clearClouds(){
    cloud->clear();
    groundCloud->clear();
    obstacleCloud->clear();
    laserCloud->clear();
}

void RGBD_Proc::publishCloud(const PcROS_Ptr& cloud_msg){
    PcROS tempCloud;
    pcl::toROSMsg(*groundCloud, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = cloud_msg->header.frame_id;
    if (ground_pub.getNumSubscribers() != 0)
        ground_pub.publish(tempCloud);

    pcl::toROSMsg(*obstacleCloud, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = cloud_msg->header.frame_id;
    if (obstacle_pub.getNumSubscribers() != 0)
        obstacle_pub.publish(tempCloud);

    pcl::toROSMsg(*laserCloud, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = cloud_msg->header.frame_id;
    if (laser_pub.getNumSubscribers() != 0)
        laser_pub.publish(tempCloud);
}

void RGBD_Proc::filterCloud(PcXYZ_Ptr cloud){
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    PcXYZ _cloud;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leafSize, 
                    leafSize, 
                    leafSize);
    sor.filter(_cloud);
    *cloud = _cloud;

    // Build a passthrough filter to remove spurious NaNs and scene background
    pcl::PassThrough<pcl::PointXYZ> passUpward;
    passUpward.setInputCloud(cloud);
    passUpward.setFilterFieldName(upward_axis_name);
    passUpward.setFilterLimits(bottomLimit, topLimit);
    passUpward.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> passForward;
    passForward.setInputCloud(cloud);
    passForward.setFilterFieldName(forward_axis_name);
    passForward.setFilterLimits(backLimit, frontLimit);
    passForward.filter(*cloud);
}

int main (int argc, char** argv) {
	ros::init (argc, argv, "rgbd_proc");
	RGBD_Proc *oDetect = new RGBD_Proc();
	ros::spin();
}