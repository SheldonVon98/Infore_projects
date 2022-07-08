#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <iostream>

#define FIX_QRCODE
using namespace std;
const float deg2rad = 3.1415926/180.0;
const float rad2angle = 180.0/3.1415926;
const float ratio = 1.0/500.0;

void poseCallback(const geometry_msgs::Point32 position){
    string fixName = "cam";
    string objName = "QR";
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    float angle = (position.y-90) * deg2rad;
    float x = -position.x*ratio;
    float y = position.z*ratio;

#ifdef FIX_QRCODE
    if(y!=0){
        float s = sqrt(x*x+y*y);
        float alpha = atan(x/y)-angle;
        x = s*cos(alpha);
        y = s*sin(alpha);
    }
    cout << "x: " << int(x/ratio) << "   y: " << int(y/ratio) << "   angle: " << angle*rad2angle << endl << endl;
#endif
    
    q.setEuler(0, 0, angle);
    transform.setOrigin(tf::Vector3(x, y, 0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, 
                                        ros::Time::now(), 
                                        fixName, 
                                        objName));
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "sv_tf_broadcaster");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/pose_publisher", 10, &poseCallback);
    ros::spin();
    return 0;
}
