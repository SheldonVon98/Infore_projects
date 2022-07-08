#include "ground_seg.h"

GroundSeg::GroundSeg(
    Eigen::Vector3f upward_axis, 
    PcXYZ_Ptr src,
    PcXYZ_Ptr ground,
    PcXYZ_Ptr obstacle){
    // Get params from launch file.
    this->upward_axis = upward_axis;
    this->srcCloud = src;
    this->groundCloud = ground;
    this->obstacleCloud = obstacle;
    getParams();
}

GroundSeg::~GroundSeg(){
}

void GroundSeg::initSAC(){
    if(isSACInitialized) return;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne; 
    // normal
    ne.setSearchMethod(tree);
    ne.setInputCloud(srcCloud);
    ne.setKSearch(neKSearch);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(segNormalDistanceWeight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(segMaxIterations);
    seg.setDistanceThreshold(segDistanceThreshold);
    seg.setInputCloud(srcCloud);
    seg.setInputNormals(cloud_normals);
    seg.setAxis(upward_axis);
    // angle bias
    seg.setEpsAngle(pcl::deg2rad(segEpsAngle));
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Model coefficients: " << coefficients_plane->values[0] << " " 
						<<coefficients_plane->values[1] << " "
						<<coefficients_plane->values[2] << " " 
						<<coefficients_plane->values[3] <<std::endl;
    isSACInitialized = true;
}

void GroundSeg::getParams(){
    ros::param::get(ros::this_node::getName()+"/neKSearch", neKSearch);
    ros::param::get(ros::this_node::getName()+"/segDistanceThreshold", segDistanceThreshold);
    ros::param::get(ros::this_node::getName()+"/segMaxIterations", segMaxIterations);
    ros::param::get(ros::this_node::getName()+"/segEpsAngle", segEpsAngle);
    ros::param::get(ros::this_node::getName()+"/segNormalDistanceWeight", segNormalDistanceWeight);
}

void GroundSeg::removeGround(){
    #ifndef DIRECT_REMOVE_GROUND
        initSAC();
    #endif
    pcl::PointXYZ point;
    for (size_t i = 0; i < srcCloud->size(); i++){
        point.x = srcCloud->points[i].x;
        point.y = srcCloud->points[i].y;
        point.z = srcCloud->points[i].z;
        // Compute the distance from the point to the segmentation plane.
        #ifndef DIRECT_REMOVE_GROUND
            double dist2plane = coefficients_plane->values[0] * point.x +
                                coefficients_plane->values[1] * point.y +
                                coefficients_plane->values[2] * point.z +
                                coefficients_plane->values[3];
            if(dist2plane > 0.1){
                // If the point is above the plane with a margin of 0.1,
                // then it belongs to the obstacle cloud.
        #else
            if(point.z > -0.5){
        #endif
                obstacleCloud->push_back(point);
            } else {
                groundCloud->push_back(point);
            }

    }
}
 