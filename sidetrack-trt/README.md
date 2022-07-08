# SideTrack-trt

vision-based lane & side tracking algorithm written in CPP with TensorRT using fscn model.

## Installation
Clone this repository into your catkin workspace and execute standard build:
```
catkin_make
```
## Usage
### Engine
Two default trained engine is provided in `engines/` folder. If your want to use your own engine, please place the engine in the above mentioned folder and edit `launch/front.launch`:
```
<launch>
    <node pkg="side_track" type="side_track_detect" name="side_track_detect" output="screen">
        <param name="engine" type="str" value="path_to_your_engine"/>
        ...
    </node>
    ...
</launch>
```
To train your own engine, please use this repository:
```
http://gitlab.infore.com/fengds/sidetrack-pytorch.git
```
### Run
Please specify the image input node in the `launch/front.launch` file, then you can launch with the following command:
```
roslaunch side_track front.launch
```
Echo to see the output message:
```
rostopic echo /front_msgs
```