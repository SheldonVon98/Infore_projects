# PSDet

Parking Slot Detection & Positioning Tool

## Installation

```
git clone http://10.48.21.42/fengds/psdet.git
```

## Usage

```
roslaunch psdet_linebase PSDetLB.launch
```

## Description

![overviw](/assets/overview.png)

## Topic

### Subscribe

* /sur_camera/image_raw: [sensor_msgs::Image] Input birdview image

### Publish

* /PSDetLB/detImg: [sensor_msgs::Image] output birdview detection image
