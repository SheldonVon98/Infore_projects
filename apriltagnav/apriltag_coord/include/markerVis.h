#ifndef MARKER_VIS_H
#define MARKER_VIS_H
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>


using namespace std;

class MarkerVis{
private:
    ros::Publisher marker_pub;
    ros::NodeHandle nh;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker makeMarker(cv::Point2d point, 
                                        string name, 
                                        int action);

public:
    MarkerVis(std::string topicName);
    ~MarkerVis();
    void convertPoints2mArr(cv::Point2d position);
    void removePoints2mArr();
    void publish();
};

#endif