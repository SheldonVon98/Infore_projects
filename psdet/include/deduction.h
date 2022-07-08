#ifndef DEDUCTION_H
#define DEDUCTION_H

#include <iostream>
#include "yololayer.h"
#include <opencv2/opencv.hpp>
#include "NvInfer.h"
#include "macros.h"

/***************************************************
              left side
            p1----------p4
            |           |
    car     |     c     |  back side
            |           |
            p2----------p3
              right side

class 0 --- psLinesL => park slot lines /
class 1 --- psLinesR => park slot lines \
class 2 --- psLinesH => park slot lines -
class 3 --- psLinesV => park slot lines |
class 4 --- psLinesAv => park slot available
class 5 --- psLinesnAv => park slot unavailable
****************************************************/

#define psLinesL    0
#define psLinesR    1
#define psLinesH    2
#define psLinesV    3
#define psLinesAv   4
#define psLinesnAv  5

#define PS_CENTER_RADIUS        26
#define PS_CENTER_THICKNESS     5
#define PSNAV_COLOR             0, 0, 255
#define PSAV_COLOR              255, 0, 0
#define PS_LINE_REPEL_TH        200
#define PS_LINE_COLOR           0, 255, 0
#define PS_ENTRY_LINE_COLOR     200, 200, 50
#define PS_LINE_THICKNESS       6

using namespace cv;
using namespace std;

enum PSType{
    Invalid,
    Available,
    Unavailable,
};

typedef struct PSLine{
    Point start, end;  
};

typedef struct ParkingSlot{
    PSType type;
    Point center;
    PSLine leftLine;
    PSLine rightLine;
    PSLine backLine;
    PSLine frontLine;
    float conf;
};

class Deduction{
private:
    float ratio_w, ratio_h;
    vector<ParkingSlot> pss;
    Mat *image;
public:
    Deduction(/* args */);
    void convert(Mat* img, 
        vector<Yolo::Detection> res);
    /*
    *   replel line prediction:
    *       Parking slot line has four
    *       classes and thus can not be
    *       process by NMS.
    *       This function will remove the
    *       redundant psLines in a small 
    *       area which is specified by
    *       PS_LINE_REPEL_TH
    */
    vector<Yolo::Detection> repelClosePreds(vector<Yolo::Detection> res);
    /*
    *   Process detection W.R.T parking 
    *   slot center point.
    */
    ParkingSlot psAvDeduction(Yolo::Detection psCenter, 
                        vector<Yolo::Detection> psLines);
    /*
    *   get PSLine from yolo detection
    */
    PSLine getLine(Yolo::Detection);
    Point getBoxCenter(float *bbox);
    void visualization();
    void circleWithConf(Point center, float conf, Scalar color);
    ~Deduction();
};

#endif