# AprilTagNav 二维码定位功能模块

本项目提供基于apriltag的rgb图像定位功能，适用于清扫机器人回位充电的场景。本项目针对次场景增加了停障功能。

## 代码位置
```
http://10.48.21.42/aiteam/apriltagnav.git
```

## 依赖

* ira_laser_tools

## 使用方法
### 编译
```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone http://10.48.21.42/aiteam/apriltagnav.git
cd ..
catkin_make_isolated
source devel/setup.sh
```
### 启动
```
roslaunch apriltag_ros detect.launch
```
## 代码原理
本项目依赖于apriltag和apriltag_ros, 其代码地址如下：
```
https://github.com/AprilRobotics/apriltag.git
https://github.com/AprilRobotics/apriltag_ros.git
```

### 二维码定位功能
* pose_publisher_trig.cpp
1. 通过Subscribe apriltag_ros中的Topic：/tag_detections，我们可以得到相机坐标系下的二维码坐标及姿态（四元数）。
2. 通过四元数获得RPY后，在仅考虑一维的情况下，通过三角函数关系把数据从相机坐标系变换到二维码坐标系下。

### 障碍物检测功能
* 障碍物输入
  
apriltagnav/apriltag_ros/launch/detect.launch
```
<launch>
    <node pkg="ira_laser_tools" name="laserscan_multi_merger" type="laserscan_multi_merger" output="screen">
        ...
        <param name="laserscan_topics" value ="/scan /rgbd_pc_id1/scan" />
    </node>
    ...
</launch>
```
障碍物可以同时接受激光雷达LaserScan和RGB-D点云PointCloud的数据，本功能通过Subscribe：/scan /rgbd_pc_id1/scan两个Topic来获得数据。

* obstacleDetect.cpp
1. 通过Subscribe Topic：/cloud_merge,可以检测机器人到二维码之间的障碍物。
2. 本功能仅支持停障（有无障碍物），不提供障碍物的具体信息（位置，种类等）。

### 调试可视化功能
* markerVis.h 
1. 调试时可通过Subscribe Topic：/aprilnav/pose_marker来获得可视化信息。

## 通信协议
### 输入
| 话题名称                          | 类型                      | 描述               | 
| -----------                      | -----------              | -----------       |
| /camera_id1/color/image_raw      | sensor_msgs::Image        | 摄像头图像             |
| /rgbd_pc_id1/scan                | sensor_msgs::LaserScan    | RGB-D点云投影到二维平面的数据              |
| /scan                            | sensor_msgs::LaserScan    | 激光雷达数据              |

### 输出
| 话题名称                          | 类型                      | 描述               | 
| -----------                      | -----------              | -----------       |
| /aprilnav/pose_publisher         | common_msgs::autoCharge   | 回位充电定位数据              |
| /aprilnav/pose_marker            | visualization_msgs::MarkerArray   | 回位充电定位数据              |

##  message类型 common_msgs/autoCharge
message可在以下repo找到：http://10.48.21.42/50-cleanrobot/thirdparty/common_msgs.git
```
float32 bias        # 二维码横向偏差
float32 distance    # 二维码距离
float32 angle       # 二维码角度
bool    stop        # 是否停障
```