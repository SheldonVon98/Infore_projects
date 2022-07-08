#ifndef GROUND_SEG_H
#define GROUND_SEG_H

#define DIRECT_REMOVE_GROUND

#include "common.h"
using namespace std;

class GroundSeg{
private:
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
    pcl::ModelCoefficients::Ptr coefficients_plane{new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers_plane{new pcl::PointIndices};

    PcXYZ_Ptr srcCloud;
    PcXYZ_Ptr groundCloud;
    PcXYZ_Ptr obstacleCloud;
    

    float segDistanceThreshold;
    float segEpsAngle;
    float segNormalDistanceWeight;
    int segMaxIterations;
    int neKSearch;
    Eigen::Vector3f upward_axis;
    bool isSACInitialized = false;
    
public:
    GroundSeg(
        Eigen::Vector3f upward_axis, 
        PcXYZ_Ptr src,
        PcXYZ_Ptr ground,
        PcXYZ_Ptr obstacle);
    ~GroundSeg();
    void initSAC();
    void getParams();
    void removeGround();
};




#endif