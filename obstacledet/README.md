# 障碍物检测功能

本项目提供基于激光雷达与点云融合的障碍物检测功能，适用于清扫机器人避障与绕障的场景。

![overviw](/assets/1.png)
![overviw](/assets/2.png)

## 代码位置
```
http://10.48.21.42/aiteam/obstacledet.git
```

## 使用方法
### 编译
```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone http://10.48.21.42/aiteam/obstacledet.git
cd ..
catkin_make
source devel/setup.sh
```

### 启动
```
roslaunch obstacle_det detect.launch
```

## 代码原理

### 贴墙检测功能
方法一：
1. 裁剪点云ROI，降采样ROI中的点云并栅格化点云。
2. 用 RANSAC 找到法线向上的平面，以此平面作为地面。
3. 上移该平面（以保证地面点全部滤除），滤除平面以下的点。
4. 把非地面点投影到平面上，发布点云
方法一：
1.水平安装摄像头，以一定高度未阈值，阈值以下的是地面。

### 调试可视化功能
1. 调试时可通过Subscribe Topic：/rgbd_pc/obstacle来获得可视化障碍物点云信息。
1. 调试时可通过Subscribe Topic：/rgbd_pc/ground来获得可视化地面点云信息。


## 通信协议
### 输入
| 话题名称                          | 类型                      | 描述               | 
| -----------                      | -----------              | -----------       |
| /camera$/depth/points            | sensor_msgs/Image        | 点云数据        |

### 输出
| 话题名称                       | 类型                              | 描述               | 
| -----------                   | -----------                      | -----------       |
| /rgbd_pc/obstacle             | sensor_msgs::PointCloud2         | 调试用障碍物点云     |
| /rgbd_pc/ground               | sensor_msgs::PointCloud2         | 调试用地面点云       |
| /rgbd_pc/laserCloud           | sensor_msgs::PointCloud2         | 投影到二维平面的点云  |
