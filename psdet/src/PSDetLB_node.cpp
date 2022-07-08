#include <ros/ros.h>
#include "PSDetLB.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "PSDetLB");
    PSDetectLB psDetector;
    ros::spin();
    return 0;
}
