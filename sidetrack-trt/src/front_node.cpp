#include "common.h"
#include "side_lane_detect.h"

int main(int argc, char **argv){
    std::string ImageSubNodeName, MsgPubNodeName, ImagePubNodeName;
    ROS_INFO("Side Track vision initializing.");
    ros::init(argc, argv, "front_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    SLDetect *sld = new SLDetect();
    ros::param::get(ros::this_node::getName()+"/ImageSubNode", ImageSubNodeName);
    ROS_INFO_STREAM("Subscribed to " << ImageSubNodeName << ".");
    image_transport::Subscriber sub = it.subscribe(ImageSubNodeName, 1, &SLDetect::rosCallback, sld);
    ros::param::get(ros::this_node::getName()+"/MsgPubNode", MsgPubNodeName);
    ROS_INFO_STREAM("Side Track results publish to " << MsgPubNodeName << ".");
    sld->setMsgPublisher(n.advertise<side_track::front_msgs>(MsgPubNodeName, 1));
    ros::param::get(ros::this_node::getName()+"/ImagePubNode", ImagePubNodeName);
    ROS_INFO_STREAM("Side Track image results publish to " << ImagePubNodeName << ".");
    sld->setImgPublisher(it.advertise(ImagePubNodeName, 1));
    ROS_INFO("Side Track vision detection ROS unit started.");
    ros::spin();
    return 0;
}