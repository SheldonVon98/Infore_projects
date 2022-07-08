#include "deduction.h"


Deduction::Deduction(){
}

void Deduction::convert(Mat* img, vector<Yolo::Detection> res){
    image = img;
    ratio_w = Yolo::INPUT_W / (img->cols * 1.0);
    ratio_h = Yolo::INPUT_H / (img->rows * 1.0);
    res = repelClosePreds(res);

    pss.clear();
    vector<Yolo::Detection> psLines, psAv, psNAv;
    for(size_t i=0; i<res.size(); i++){
        if((int)res[i].class_id == psLinesAv){
            psAv.push_back(res[i]);
        } else if((int)res[i].class_id == psLinesnAv){
            psNAv.push_back(res[i]);
        } else {
            psLines.push_back(res[i]);
        }
    }

    for(size_t i=0; i<psAv.size(); i++){
        ParkingSlot ps = psAvDeduction(psAv[i], psLines);
        if(ps.type != Invalid) pss.push_back(ps);
    }


    for(size_t i=0; i<psNAv.size(); i++){
        ParkingSlot psnav_;
        psnav_.center = getBoxCenter(psNAv[i].bbox);
        psnav_.type = Unavailable;
        psnav_.conf = psNAv[i].conf;
        pss.push_back(psnav_);
    }
}

vector<Yolo::Detection> Deduction::repelClosePreds(vector<Yolo::Detection> res){

    vector<Yolo::Detection> repelRes;
    return res;
}

bool compareMethod(pair<double, Yolo::Detection> i1, 
        pair<double, Yolo::Detection> i2){
    return (i1.first < i2.first);
}

ParkingSlot Deduction::psAvDeduction(Yolo::Detection psCenter, 
                        vector<Yolo::Detection> psLines){

    ParkingSlot psav_;
    psav_.center = getBoxCenter(psCenter.bbox);
    psav_.type = Available;
    psav_.conf = psCenter.conf;

    // Find neighbouring psLines
    if(psLines.size() >= 2){
        vector<pair<double, Yolo::Detection>> dist;
        Point psCt = getBoxCenter(psCenter.bbox);
        for(size_t i=0; i<psLines.size(); i++)
            dist.push_back(make_pair(norm(psCt-getBoxCenter(psLines[i].bbox)), psLines[i]));
        
        sort(dist.begin(), dist.end(), compareMethod);
        if(dist[1].first < PS_LINE_REPEL_TH){

            psav_.leftLine = getLine(dist[0].second);
            psav_.rightLine = getLine(dist[1].second);

            PSLine frontLine, backLine;
            frontLine.start = psav_.leftLine.start;
            frontLine.end = psav_.rightLine.start;
            backLine.start = psav_.leftLine.end;
            backLine.end = psav_.rightLine.end;
            psav_.frontLine = frontLine;
            psav_.backLine = backLine;
        }else {
            psav_.type = Invalid;
        }
    } else {
        psav_.type = Invalid;
    }
    return psav_;
}


PSLine Deduction::getLine(Yolo::Detection det){
    float *bbox = det.bbox;
    int cls = det.class_id;
    int x1, y1, x2, y2;
    PSLine line;

    if(cls == psLinesL){
        x1 = (bbox[0] - bbox[2] / 2.f) / ratio_w;
        y1 = (bbox[1] + bbox[3] / 2.f) / ratio_h;
        x2 = (bbox[0] + bbox[2] / 2.f) / ratio_w;
        y2 = (bbox[1] - bbox[3] / 2.f) / ratio_h;
    } else if(cls == psLinesR){
        x1 = (bbox[0] - bbox[2] / 2.f) / ratio_w;
        y1 = (bbox[1] - bbox[3] / 2.f) / ratio_h;
        x2 = (bbox[0] + bbox[2] / 2.f) / ratio_w;
        y2 = (bbox[1] + bbox[3] / 2.f) / ratio_h;
    } else if(cls == psLinesH){
        x1 = (bbox[0] - bbox[2] / 2.f) / ratio_w;
        y1 = bbox[1] / ratio_h;
        x2 = (bbox[0] + bbox[2] / 2.f) / ratio_w;
        y2 = bbox[1] / ratio_h;
    } else if(cls == psLinesV){
        x1 = bbox[0] / ratio_w;
        y1 = (bbox[1] - bbox[3] / 2.f) / ratio_h;
        x2 = bbox[0] / ratio_w;
        y2 = (bbox[1] + bbox[3] / 2.f) / ratio_h;
    }
    line.start = Point(x1, y1);
    line.end = Point(x2, y2);
    return line;
}

Point Deduction::getBoxCenter(float *bbox){
/*
    bbox => x, y, w, h
*/
    return Point(bbox[0] / ratio_w, bbox[1] / ratio_h);
}

void Deduction::visualization(){
    for(size_t i=0; i<pss.size(); i++){
        if(pss[i].type == Unavailable){
            circleWithConf(pss[i].center, 
                        pss[i].conf, 
                        Scalar(PSNAV_COLOR));
        } else {
            circleWithConf(pss[i].center, 
                        pss[i].conf, 
                        Scalar(PSAV_COLOR));
            line(*image, 
                pss[i].leftLine.start, 
                pss[i].leftLine.end,
                Scalar(PS_LINE_COLOR),
                PS_LINE_THICKNESS);
            line(*image, 
                pss[i].rightLine.start, 
                pss[i].rightLine.end,
                Scalar(PS_LINE_COLOR),
                PS_LINE_THICKNESS);
            line(*image, 
                pss[i].frontLine.start, 
                pss[i].frontLine.end,
                Scalar(PS_ENTRY_LINE_COLOR),
                PS_LINE_THICKNESS);
            line(*image, 
                pss[i].backLine.start, 
                pss[i].backLine.end,
                Scalar(PS_LINE_COLOR),
                PS_LINE_THICKNESS);
        }
    }
}

void Deduction::circleWithConf(Point center, float conf, Scalar color){
    circle(*image, 
        center, 
        PS_CENTER_RADIUS, 
        color, 
        PS_CENTER_THICKNESS);
    std::stringstream stream;
    stream << (int)(conf*100) << "%";
    center.x -= 16;
    center.y += 8;
    putText(*image, 
        stream.str(), 
        center, 
        cv::FONT_HERSHEY_PLAIN, 
        1.2, 
        cv::Scalar(0xFF, 0xFF, 0xFF), 
        2);
}

Deduction::~Deduction(){
}
