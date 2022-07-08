#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include <pluginlib/class_list_macros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
#include <list>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


using namespace std;
typedef list<float> LISTF;


class PosePublisher{

private:
    ros::NodeHandle n_;
    // ros::Publisher pub_o;
    // ros::Publisher pub_t;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    geometry_msgs::Point32 output;
    float w, x, y, z;
    float filter_x, filter_y, filter_z, filter_roll, filter_pitch, filter_yaw;
    double roll, pitch, yaw;
    float p_x, p_y, p_z;
    float t_x, t_y, t_z, r_x, r_y, r_z;
    LISTF list_x, list_y, list_z, list_roll, list_pitch, list_yaw;

    // std::string DEVICE;

public:
    PosePublisher(/* args */);
    void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray &input);
    float filter_o(LISTF l_);
    float filter_d(LISTF l_);
    ~PosePublisher(){}
};

PosePublisher::PosePublisher(/* args */){

    // ros::NodeHandle& pnh = getPrivateNodeHandle();
    // pnh.getParam("DEVICE", DEVICE);

    std::string DEVICE = "";

    // pub_o = n_.advertise<nav_msgs::Odometry>("/odometry_" + DEVICE, 1);
    // pub_t = n_.advertise<TagBundleArray>("/detection_tag_" + DEVICE, 1);
    pub_ = n_.advertise<geometry_msgs::Point32>("/pose_publisher", 1);

    sub_ = n_.subscribe("/tag_detections", 1, &PosePublisher::tagDetectionsCallback, this);

    
}

void PosePublisher::tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray &input){

        nav_msgs::Odometry odometry;
        // TagBundleArray tag_array;
        // TagBundle tag_;
        
        for (int i = 0; i < int(input.detections.size()); ++i){
            
            // for (int j = 0; j <input.detections[i].id.size(); ++j)
            // {
            //     cout << "input.detections.id " << input.detections[i].id[0] << endl;
                
            // }
            
            p_x = input.detections[i].pose.pose.pose.position.x;
            p_y = input.detections[i].pose.pose.pose.position.y;
            p_z = input.detections[i].pose.pose.pose.position.z;

            w = input.detections[i].pose.pose.pose.orientation.w;
            x = input.detections[i].pose.pose.pose.orientation.x;
            y = input.detections[i].pose.pose.pose.orientation.y;
            z = input.detections[i].pose.pose.pose.orientation.z;

            tf::Quaternion quat;
            tf::quaternionMsgToTF(input.detections[i].pose.pose.pose.orientation, quat);
            
            // 二维码在相机坐标系下 =》相机在二维码坐标系下
            // 点在坐标系逆旋转后的坐标
            // q_p(0, p_x, p_y, p_z),   q(w, x, y, z), q_1(w, -x, -y, -z)
            // q_p' =  q * q_p * q_1
            
            Eigen::Quaterniond q_1 = Eigen::Quaterniond(w, -x, -y, -z).normalized();
            Eigen::Vector3d  p = Eigen::Vector3d(p_x, p_y, p_z);
            Eigen::Vector3d p2;
            p2 = q_1 * p;

            // cout<< "p2 "<< p2 <<endl;

            odometry.header.frame_id = "odometry" ;
            odometry.child_frame_id = "pose";

            
            // 旋转顺序为R P Y, 旋转轴不固定
            // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw) 求解顺序为 R P Y, 得到旋转轴不固定的旋转角度
            // tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll); 求解顺序为 Y P R, 得到旋转轴固定 分别旋转的角度            
            tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);
            
            int list_size = 20;
            
            //滤波
            if (list_x.size() >= list_size)
            {
                list_x.pop_front();
            }
            list_x.push_back(p2[0]);
            filter_x = filter_d(list_x);

            if (list_y.size() >= list_size)
            {
                list_y.pop_front();
            }
            list_y.push_back(p2[1]);
            filter_y = filter_d(list_y);

            if (list_z.size() >= list_size)
            {
                list_z.pop_front();
            }
            list_z.push_back(p2[2]);
            filter_z = filter_d(list_z);
            
            if (list_roll.size() >= list_size)
            {
                list_roll.pop_front();
            }
            list_roll.push_back(roll);
            filter_roll = filter_o(list_roll);

            if (list_pitch.size() >= list_size)
            {
                list_pitch.pop_front();
            }
            list_pitch.push_back(pitch);
            filter_pitch = filter_o(list_pitch);

            if (list_yaw.size() >= list_size)
            {
                list_yaw.pop_front();
            }
            list_yaw.push_back(yaw);
            filter_yaw = filter_o(list_yaw);            

            t_x = filter_x ;    //mm
            t_y = filter_y ;
            t_z = filter_z ;

            roll = filter_roll/M_PI * 180;
            yaw =  filter_yaw/M_PI * 180;
            pitch = filter_pitch/M_PI * 180;

            odometry.pose.pose.orientation = input.detections[i].pose.pose.pose.orientation;
            odometry.pose.pose.position.x = t_x;
            odometry.pose.pose.position.y = t_y;
            odometry.pose.pose.position.z = t_z;
            
            // tag_.tag_id = input.detections[i].id[0];
            // tag_.o_x = roll;
            // tag_.o_y = pitch;
            // tag_.o_z = yaw;

            // tag_.p_x = t_x;
            // tag_.p_y = t_y;
            // tag_.p_z = t_z;
            
            // cout << "r_x: "<<  roll << " r_y: "<< pitch << " r_z: " << yaw << "t_x: "<< t_x << " t_y: "<< t_y << " t_z: " << t_z <<endl; 
                
        
        // tag_array.tagbundles.push_back(tag_);
        
            
        }       
        // cout << "r_x: " <<  roll 
        //     << " r_y: "<< pitch 
        //     << " r_z: " << yaw 
        //     << "t_x: "<< t_x 
        //     << " t_y: "<< t_y 
        //     << " t_z: " << t_z 
        //     <<endl; 
        
        //发布
        // pub_o.publish(odometry);
        // pub_t.publish(tag_array);
        output.x = t_x*1000;
        output.z = t_z*1000;
        output.y = pitch;
        
        //发布
        pub_.publish(output);
}

float PosePublisher::filter_d(LISTF l_)
{
    /*
  对数据进行滤波处理，取连续9组数据排序，新数据与前九组数据的中位数进行比较，差值大，取中位数
  */
    float temp = 0;
    float middle = 0;
    vector<float> v_;
    v_.assign(l_.begin(), l_.end());
    temp = v_[v_.size() - 1];
    sort(v_.begin(), v_.end());
    middle = v_[int(v_.size() / 2)];
    if (abs(temp - middle) > 0.05)
    {
        temp = middle;
    }
    return temp;
}

float PosePublisher::filter_o(LISTF l_)
{
    /*
  对数据进行滤波处理，取连续9组数据排序，新数据与前九组数据的中位数进行比较，差值大，取中位数
  */
    float temp = 0;
    float middle = 0;
    vector<float> v_;
    v_.assign(l_.begin(), l_.end());
    temp = v_[v_.size() - 1];
    sort(v_.begin(), v_.end());
    middle = v_[int(v_.size() / 2)];
    if (abs(temp - middle) > middle*0.05)
    {
        temp = middle;
    }
    return temp;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "pose_publisher");
    PosePublisher ppObj;
    ros::spin();
    return 0;
}