# RTSPStream2ROS

Retrieve RTSP stream and publish to ROS node.

## Build
Simply clone into ros workspace and:
```
catkin_make
```

## Run
```
roslaunch rtsp_stream stream.launch
```
## Nodes
### Calibrated Image Node
```
/rtsp_stream/image
```
### Raw Image Node
```
/rtsp_stream/image_raw
```