# 激光贴墙检测功能

本项目提供基于激光雷达的墙面定位功能，适用于清扫机器人贴墙清扫作业的场景。

![overviw](/assets/thumnail.png)


## 代码位置
```
http://10.48.21.42/aiteam/laser_stickwall.git
```

## 使用方法
### 编译
```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone http://10.48.21.42/aiteam/laser_stickwall.git
cd ..
catkin_make
source devel/setup.sh
```

### 启动
```
roslaunch laser_stickwall detect_wall.launch
```

## 代码原理

### 贴墙检测功能

1. 截取激光雷达的ROI区域，目前设定值是以机器人雷达为原点的前方2m、后方0.5m、左右各1m。具体可以在laser_stickwall/launch/detect_wall.launch中设置。
2. 墙体粗匹配。利用ROI中所有的点进行线性回归拟合出粗略的墙体，通过点线距离误差判断拟合的直线是否合格。
3. 障碍物去除。从粗匹配的墙点中的最前方的点开始计算，去除障碍物（距离拟合直线过大的点），得到有效墙面点。
4. 墙面细匹配。对有效墙面点再次进行线性回归，拟合出墙面直线。
5. 利用墙面直线均匀地得到导航点。

### 调试可视化功能
1. 调试时可通过Subscribe Topic：/elevator/markerArray来获得可视化信息。
1. 调试时可通过Subscribe Topic：/elevator/image来获得可视化图像信息。

### 约束与条件
1. 在贴墙任务的起点位置，机器人应贴近墙面（小于1m）。
2. 从起点到重点的墙面应该至少有2m长。

## 通信协议
### 输入
| 话题名称                          | 类型                      | 描述               | 
| -----------                      | -----------              | -----------       |
| /scan                            | sensor_msgs/LaserScan    | 激光雷达数据        |

### 输出
| 话题名称                       | 类型                              | 描述               | 
| -----------                   | -----------                      | -----------       |
| /wall_nav/wall_path           | nav_msgs::Path                   | 贴墙导航信息        |
| /wall_nav/image               | sensor_msgs::Image               | 调试用图像          |
