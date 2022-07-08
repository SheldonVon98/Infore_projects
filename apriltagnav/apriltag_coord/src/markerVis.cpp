#include "markerVis.h"

MarkerVis::MarkerVis(std::string topicName){
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(topicName, 1);
}

MarkerVis::~MarkerVis(){
}


visualization_msgs::Marker MarkerVis::makeMarker(cv::Point2d point, 
                                    string name, 
                                    int action){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = action;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
}
void MarkerVis::convertPoints2mArr(cv::Point2d position){
    markerArray.markers.clear();
    markerArray.markers.push_back(makeMarker(position, "tag", visualization_msgs::Marker::ADD));

}

void MarkerVis::removePoints2mArr(){
    markerArray.markers.clear();
    markerArray.markers.push_back(makeMarker(cv::Point2d(0, 0), "tag", visualization_msgs::Marker::DELETEALL));
}

void MarkerVis::publish(){
    marker_pub.publish(markerArray);
}

