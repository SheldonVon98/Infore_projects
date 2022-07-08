#ifndef RETRIEVE_H
#define RETRIEVE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "common.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
}

#define DECODER AV_CODEC_ID_H265

class Retrieve{
private:
    AVPacket packet;
    AVStream* stream;
    AVCodecContext* codec_ctx;
    SwsContext *img_convert_ctx;
    int video_stream_index;
    uint8_t* picture_buffer;
    AVFrame* picture;
    AVFrame* picture_rgb;
    uint8_t* picture_buffer_2;
    AVFormatContext* format_ctx;
    AVFormatContext* output_ctx;
public:
    Retrieve(std::string rtspURL);
    ~Retrieve();
    void play();
    void run(void(*callback)(AVCodecContext* codec_ctx, 
                        AVFrame* picture_rgb));
};

#endif