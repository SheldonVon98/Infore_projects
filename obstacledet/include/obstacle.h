#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "common.h"

#define STATE_DIM 4 // [x,y,v_x,v_y]//,w,h]
#define MEAS_DIM  2  // [z_x,z_y,z_w,z_h]
#define CTRL_DIM  0


class Obstacle{
private:
    cv::KalmanFilter kmf;

public:
    Obstacle(/* args */);
    ~Obstacle();
};

#endif