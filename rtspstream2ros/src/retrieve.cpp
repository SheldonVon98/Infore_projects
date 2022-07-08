#include "retrieve.h"

Retrieve::Retrieve(std::string rtspURL){
    // Open the initial context variables that are needed
    format_ctx = avformat_alloc_context();
    codec_ctx = NULL;
    // Register everything
    av_register_all();
    avformat_network_init();

    AVDictionary *opDict = NULL;
    //open RTSP--
    av_dict_set(&opDict, "rtsp_transport", "tcp", 0);
    av_dict_set(&opDict, "fflags", "nobuffer", 0);
    av_dict_set(&opDict, "flags", "low_delay", 0);
    
    if (avformat_open_input(&format_ctx, 
                            rtspURL.c_str(),
                            0, 
                            &opDict) != 0) {
        ROS_ERROR("could not open stream.");
        return;
    }

    if (avformat_find_stream_info(format_ctx, NULL) < 0) {
        return;
    }

    //search video stream
    for (int i = 0; i < format_ctx->nb_streams; i++) {
        if (format_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
            video_stream_index = i;
    }
    
    av_init_packet(&packet);

    //open output file
    output_ctx = avformat_alloc_context();

    stream = NULL;
    //start reading packets from stream and write them to file
    av_read_play(format_ctx);    //play RTSP

    // Get the codec
    AVCodec *codec = NULL;
    codec = avcodec_find_decoder(DECODER);
    if (!codec) {
        exit(1);
    }
    // Add this to allocate the context by codec
    codec_ctx = avcodec_alloc_context3(codec);

    codec_ctx->thread_count = 1;
    avcodec_get_context_defaults3(codec_ctx, codec);
    avcodec_copy_context(codec_ctx, format_ctx->streams[video_stream_index]->codec);

    if (avcodec_open2(codec_ctx, codec, NULL) < 0)
        exit(1);

    img_convert_ctx = sws_getContext(codec_ctx->width, codec_ctx->height,
            codec_ctx->pix_fmt, codec_ctx->width, codec_ctx->height, AV_PIX_FMT_BGR24,
            SWS_BICUBIC, NULL, NULL, NULL);

    int size = avpicture_get_size(AV_PIX_FMT_YUV420P, codec_ctx->width,
            codec_ctx->height);
    picture_buffer = (uint8_t*) (av_malloc(size));
    picture = av_frame_alloc();
    picture_rgb = av_frame_alloc();

    int size2 = avpicture_get_size(AV_PIX_FMT_BGR24, codec_ctx->width,
            codec_ctx->height);
    picture_buffer_2 = (uint8_t*) (av_malloc(size2));
    avpicture_fill((AVPicture *) picture, picture_buffer, AV_PIX_FMT_YUV420P,
            codec_ctx->width, codec_ctx->height);
    avpicture_fill((AVPicture *) picture_rgb, picture_buffer_2, AV_PIX_FMT_BGR24,
            codec_ctx->width, codec_ctx->height);
}

void Retrieve::play(){
    cv::Mat rFrame;

    while (av_read_frame(format_ctx, &packet) >= 0) {

        if (packet.stream_index == video_stream_index) {    //packet is video
            // std::cout << "2 Is Video" << std::endl;
            if (stream == NULL) {    //create stream in file
                // std::cout << "3 create stream" << std::endl;
                stream = avformat_new_stream(output_ctx,
                        format_ctx->streams[video_stream_index]->codec->codec);
                avcodec_copy_context(stream->codec,
                        format_ctx->streams[video_stream_index]->codec);
                stream->sample_aspect_ratio =
                        format_ctx->streams[video_stream_index]->codec->sample_aspect_ratio;
            }
            int check = 0;
            packet.stream_index = stream->id;
            // std::cout << "4 decoding" << std::endl;
            int result = avcodec_decode_video2(codec_ctx, picture, &check, &packet);
            // std::cout << "Bytes decoded " << result << " check " << check << std::endl;
            sws_scale(img_convert_ctx, picture->data, picture->linesize, 0,
                        codec_ctx->height, picture_rgb->data, picture_rgb->linesize);

            // std::cout << codec_ctx->width  << " " << codec_ctx->height << std::endl;
            cv::Mat mat(codec_ctx->height, codec_ctx->width, CV_8UC3, picture_rgb->data[0], picture_rgb->linesize[0]);
            cv::resize(mat, rFrame, cv::Size(720, 480));
            cv::imshow("frame", rFrame);
            cv::waitKey(1);
        }
        av_free_packet(&packet);
        av_init_packet(&packet);
    }
}

void Retrieve::run(void(*callback)(AVCodecContext* codec_ctx, AVFrame* picture_rgb)){
    while (av_read_frame(format_ctx, &packet) >= 0) {
        if (packet.stream_index == video_stream_index) { //packet is video
            if (stream == NULL) { //create stream in file
                stream = avformat_new_stream(output_ctx,
                        format_ctx->streams[video_stream_index]->codec->codec);
                avcodec_copy_context(stream->codec,
                        format_ctx->streams[video_stream_index]->codec);
                stream->sample_aspect_ratio =
                        format_ctx->streams[video_stream_index]->codec->sample_aspect_ratio;
            }
            int check = 0;
            packet.stream_index = stream->id;
            int result = avcodec_decode_video2(codec_ctx, picture, &check, &packet);
            sws_scale(img_convert_ctx, picture->data, picture->linesize, 0,
                        codec_ctx->height, picture_rgb->data, picture_rgb->linesize);
            (*callback)(codec_ctx, picture_rgb);
        }
        av_free_packet(&packet);
        av_init_packet(&packet);
    }
}

Retrieve::~Retrieve(){
    av_free(picture);
    av_free(picture_rgb);
    av_free(picture_buffer);
    av_free(picture_buffer_2);

    av_read_pause(format_ctx);
    avio_close(output_ctx->pb);
    avformat_free_context(output_ctx);
}
