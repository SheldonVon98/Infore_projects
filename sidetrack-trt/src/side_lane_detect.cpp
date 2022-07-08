#include "side_lane_detect.h"

SLDetect::SLDetect(){
#ifdef DEBUG
    ROS_INFO("Debug Mode" );
    #ifdef ALLOW_SHOW_CV
        ros::param::get(ros::this_node::getName()+"/show_cv", show_cv);
    #endif
#endif
    ROS_INFO_STREAM("Set cuda device: " << DEVICE );
    cudaSetDevice(DEVICE);

    // Read engine file
    std::string enginePath;
    ros::param::get(ros::this_node::getName()+"/engine", enginePath);

    ROS_INFO_STREAM("Reading engine file:" << enginePath);
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(enginePath, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    } else {
        ROS_ERROR("could not open plan file.");
    }

    // prepare input data ---------------------------
    ROS_INFO("Preparing input data...");
    assert(INPUT_INDEX == 0);
    assert(OUTPUT_INDEX == 1);
    // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[INPUT_INDEX], INPUT_SIZE * sizeof(float)));
    CHECK(cudaMalloc(&buffers[OUTPUT_INDEX], OUTPUT_SIZE * sizeof(float)));

    // prepare engine
    ROS_INFO("Initializing engine for inference..."); 
    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    CHECK(cudaStreamCreate(&stream));
    ROS_INFO("Initialization success.");

}

SLDetect::~SLDetect(){
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFreeHost(buffers[INPUT_INDEX]));
    CHECK(cudaFreeHost(buffers[OUTPUT_INDEX]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
}

void SLDetect::setMsgPublisher(ros::Publisher pub){
    frontMsgPub = pub;
}

void SLDetect::setImgPublisher(image_transport::Publisher pub){
    imgResPub = pub;
}

void SLDetect::msgData2buffer(const std::vector<unsigned char> data){
    float *dataPtr1 = &input_data[0*INPUT_H*INPUT_W];
    float *dataPtr2 = &input_data[1*INPUT_H*INPUT_W];
    float *dataPtr3 = &input_data[2*INPUT_H*INPUT_W];
    for(auto msgDataPtr = std::begin(data); msgDataPtr!=std::end(data);){
        *dataPtr1 = ((float)*msgDataPtr)/255.0;
        dataPtr1++;
        msgDataPtr++;
        *dataPtr2 = ((float)*msgDataPtr)/255.0;
        dataPtr2++;
        msgDataPtr++;
        *dataPtr3 = ((float)*msgDataPtr)/255.0;
        dataPtr3++;
        msgDataPtr++;
    }
}

void SLDetect::mat2buffer(cv::Mat* image){
    /*
    *   Image data format used in opencv is
    *   (height, width, channel), which does not
    *   match with pytorch image format, i.e. 
    *   (channel, height, width).
    *   Since the model is converted from pytorch,
    *   we need to manipulate the data before filling
    *   it to data buffer
    */
    // Insert three pointers to the beginning
    // of each channel in the data buffer.
    float *dataPt1 = &input_data[0*INPUT_H*INPUT_W];
    float *dataPt2 = &input_data[1*INPUT_H*INPUT_W];
    float *dataPt3 = &input_data[2*INPUT_H*INPUT_W];

    
    // Loop throught the image and pass RGB
    // channel values to the corresponding position
    // in the data buffer.
    for(int row = 0; row < INPUT_H; ++row) {
        for (int col = 0; col < INPUT_W; ++col) {
            cv::Vec3f d = image->at<cv::Vec3f>(row, col);
            *dataPt1 = d[0];
            *dataPt2 = d[1];
            *dataPt3 = d[2];
            dataPt1++;
            dataPt2++;
            dataPt3++;
        }
    }
}

void SLDetect::inference(){
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CHECK(cudaMemcpyAsync(buffers[INPUT_INDEX], input_data, INPUT_SIZE * sizeof(float), cudaMemcpyHostToDevice, stream));
    context->enqueue(BATCH_SIZE, buffers, stream, nullptr);
    CHECK(cudaMemcpyAsync(output_prob, buffers[OUTPUT_INDEX], OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}
 
std::vector<cv::Point2f> SLDetect::getContourCurve(cv::Mat singleClassImg){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point2f> contour_null;
    std::vector<cv::Vec4i> hierarchy;
    int contour_length_max = 0;
    cv::Point *contours_start=nullptr, *contours_end=nullptr;
    cv::findContours(singleClassImg, 
                    contours, 
                    hierarchy, 
                    cv::RETR_LIST, 
                    cv::CHAIN_APPROX_NONE);
    for(size_t i = 0; i< contours.size(); i++ ){
        if(contours[i].size() < MIN_CONTOURS) continue;
        // Find contours point with greatest y value (at the bottom).
        auto contours_bottom_ptr = std::max_element(begin(contours[i]), end(contours[i]),
                                [](cv::Point a, cv::Point b){return (a.y < b.y);});
        // Find contours point with smallest y value (at the bottom).
        auto contours_top_ptr = std::max_element(begin(contours[i]), end(contours[i]),
                                [](cv::Point a, cv::Point b){return (a.y > b.y);});
        // Get corresponding indices.
        int contours_top_idx = std::distance(contours[i].begin(), contours_top_ptr);
        int contours_bottom_idx = std::distance(contours[i].begin(), contours_bottom_ptr);
        // Expel contours if it's bottom is not at the bottom.
        if(contours_bottom_ptr[0].y < CONTOURS_MIN_POS) continue;
        // Expel contours if it is not long enough.
        int contour_length = (contours_bottom_ptr[0].y-contours_top_ptr[0].y);
        if(contour_length < CONTOURS_MIN_LENGTH) continue;
        // select the contour with greatest length. 
        if(contour_length_max < contour_length){
            contours_start = &contours[i][contours_top_idx];
            contours_end = &contours[i][contours_bottom_idx];
            contour_length_max = contour_length;
        }
    }
    if(contours_start && contours_end) 
        return std::vector<cv::Point2f>(contours_start, contours_end);
    else return contour_null;
}

cv::Mat polyfit(std::vector<cv::Point2f>& in_point, int n){
    // polyfit with repect to y. 
	int size = in_point.size();
	// number of variable
	int x_num = n + 1;
	cv::Mat mat_u(size, x_num, CV_64F);
	cv::Mat mat_y(size, 1, CV_64F);
	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
			mat_u.at<double>(i, j) = pow(in_point[i].y, j);
	for (int i = 0; i < mat_y.rows; ++i)
		mat_y.at<double>(i, 0) = in_point[i].x;
	// Get coefficient matrix k
	cv::Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	return mat_k;
}

cv::Mat SLDetect::curveFit(std::vector<cv::Point2f> curve, 
                        const int curveFitTop,
                        const int curveFitBottom,
                        const int curveTop,
                        const int curveBottom,
                        const int polyNum){
    // Get the pointer from the beginning of the contour point
    // where the value is greater than curveFitTop.
    auto curveStartPtr = curve.begin();
    for(; curveStartPtr!=std::end(curve); curveStartPtr++)
        if(curveStartPtr->y >= curveFitTop) break;
    // Get the pointer from the end of the contour point
    // where the value is smaller than curveFitBottom.
    auto curveEndPtr = curve.end();
    for(curveEndPtr--; curveEndPtr!=std::begin(curve); curveEndPtr--)
        if(curveEndPtr->y <= curveFitBottom) break;
    // Get the target contour point and fit with polyfit.
    std::vector<cv::Point2f> curve2fit = 
        std::vector<cv::Point2f>(curveStartPtr, curveEndPtr);
    cv::Mat curveFitCoeff = polyfit(curve2fit, polyNum);

#ifdef DEBUG
    for (int i = curveTop; i < curveBottom; i++){
        cv::Point2d ipt;
        ipt.x = 0;
		ipt.y = i;
		for (int j = 0; j <= polyNum; j++){
			ipt.x += curveFitCoeff.at<double>(j, 0)*pow(i,j);
		}
		circle(image, ipt, 1, 
            cv::Scalar(0, 200, 0), 
            cv::FILLED, 
            cv::LINE_AA);
	}
#endif
    return curveFitCoeff;
}
void SLDetect::detectStrip(cv::Mat image, 
                        const int curveFitTop,
                        const int curveFitBottom,
                        const int curveTop,
                        const int curveBottom,
                        const double steep_threshold,
                        const double smooth_threshold,
                        TrackStatus *trackStatus,
                        const int lineFitTop,
                        const int lineFitBottom,
                        const int lineTop,
                        const int lineBottom,
                        int *distance,
                        float distance_coeff){
    std::vector<cv::Point2f> contours = getContourCurve(image);
    if(contours.size()){
        cv::Mat curveFitCoeff = curveFit(contours,
                                    curveFitTop,
                                    curveFitBottom,
                                    curveTop, 
                                    curveBottom, 
                                    CURVE_FIT_POLY_N);
        
        double curvature = curveFitCoeff.at<double>(CURVE_FIT_POLY_N, 0);
        if(curvature > 0){
            if(curvature > steep_threshold) 
                *trackStatus = STEEP_RIGHT_TURN;
            else if(curvature > smooth_threshold)
                *trackStatus = SMOOTH_RIGHT_TURN;
            else *trackStatus = STRAIGHT;
        } else {
            if(-curvature > steep_threshold) 
                *trackStatus = STEEP_LEFT_TURN;
            else if(-curvature > smooth_threshold)
                *trackStatus = SMOOTH_LEFT_TURN;
            else *trackStatus = STRAIGHT;
        }
        cv::Mat lineFitCoeff = curveFit(contours,
                                    lineFitTop,
                                    lineFitBottom,
                                    lineTop, 
                                    lineBottom, 
                                    LINE_FIT_POLY_N);
        *distance = (int)((lineFitCoeff.at<double>(1, 0)*refPoint.y+\
                        lineFitCoeff.at<double>(0, 0)-\
                        refPoint.x)*distance_coeff);
    #ifdef DEBUG
    	int radiusCircle = 2, thicknessCircle = -1;
        for(uint16_t each = 0; each < contours.size(); each+=10){    
            cv::circle(image, 
                    contours[each], 
                    radiusCircle, 
                    cv::Scalar(0, 0, 200), // (B, G, R) 
                    thicknessCircle);
        }
    #endif
    } else {
        *trackStatus = UNKNOWN;
        *distance = 0;
    }
}

void SLDetect::detectLane(cv::Mat laneImg){
    detectStrip(
        laneImg,
        LANE_CURVE_FIT_TOP,
        LANE_CURVE_FIT_BOTTOM,
        LANE_CURVE_TOP, 
        LANE_CURVE_BOTTOM, 
        LANE_STATUS_STEEP_THR,
        LANE_STATUS_SMOOTH_THR,
        &trackStatus_lane,
        LANE_LINE_FIT_TOP,
        LANE_LINE_FIT_BOTTOM,
        LANE_LINE_TOP, 
        LANE_LINE_BOTTOM, 
        &distance_lane,
        LANE_DIST_COEFF
    );
}

void SLDetect::detectSide(cv::Mat sideImg){
    detectStrip(
        sideImg,
        SIDE_CURVE_FIT_TOP,
        SIDE_CURVE_FIT_BOTTOM,
        SIDE_CURVE_TOP, 
        SIDE_CURVE_BOTTOM, 
        SIDE_STATUS_STEEP_THR,
        SIDE_STATUS_SMOOTH_THR,
        &trackStatus_side,
        SIDE_LINE_FIT_TOP,
        SIDE_LINE_FIT_BOTTOM,
        SIDE_LINE_TOP, 
        SIDE_LINE_BOTTOM, 
        &distance_side,
        SIDE_DIST_COEFF
    );
}

void SLDetect::resProcess(){
    cv::Mat laneImg(INPUT_H, INPUT_W, CV_8UC1);
    cv::Mat sideImg(INPUT_H, INPUT_W, CV_8UC1);
    float *p1 = &output_prob[0*INPUT_H*INPUT_W];
    float *p2 = &output_prob[1*INPUT_H*INPUT_W];
    float *p3 = &output_prob[2*INPUT_H*INPUT_W];
    for(int row = 0; row < INPUT_H; ++row){            
        uchar *s_pixel = sideImg.data + row * sideImg.step;
        uchar *l_pixel = laneImg.data + row * laneImg.step;
        for (int col = 0; col < INPUT_W; ++col) {
            s_pixel[col] = ((*p2 > *p1)&&(*p2 > *p3)) ? 255 : 0;
            l_pixel[col] = ((*p3 > *p1)&&(*p3 > *p2)) ? 255 : 0;
            p1++; p2++; p3++;
        }
    }
    detectLane(laneImg);
    detectSide(sideImg);
#ifdef DEBUG
    cv::Mat mask = cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC3);
    mask.setTo(cv::Scalar(200, 232, 0), laneImg);
    mask.setTo(cv::Scalar(0, 232, 200), sideImg);
    addWeighted( image, 0.8, mask, 0.2, 0.0, image);
#endif
}

cv::Mat* SLDetect::imagePreprocess(cv::Mat img, int width, int height){
    // Crop out the ROI
    img = img(cv::Rect((width-INPUT_W), 
                        (height-INPUT_H), 
                        INPUT_W, 
                        INPUT_H));
#ifdef DEBUG
    image = img;
#endif
    // squeeze the data between 0~1.
    img.convertTo(img_f32, CV_32FC3, (float)1.0/255.0);
    // data normalization
    img_f32 = (img_f32 - NORM_MEAN) / NORM_STD;
    return &img_f32;
}

void SLDetect::publishMsg(){
    resMsg.lane_status = trackStatus_lane;
    resMsg.lane_dist = distance_lane;
    resMsg.side_status = trackStatus_side;
    resMsg.side_dist = distance_side;
    frontMsgPub.publish(resMsg);
}

void SLDetect::publishImg(){
    imgMsg = cv_bridge::CvImage(std_msgs::Header(), 
                                "bgr8", 
                                image).toImageMsg();
    imgResPub.publish(imgMsg);
}

void SLDetect::rosCallback(const sensor_msgs::ImageConstPtr& msg){
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat* imgPtr = imagePreprocess(cv_bridge::toCvShare(msg, "rgb8")->image, 
                                    msg->width, 
                                    msg->height);
    mat2buffer(imgPtr);
    #ifdef DEBUG
        cv::cvtColor(image, image, CV_RGB2BGR);
    #endif
    auto inferStart = std::chrono::high_resolution_clock::now();
    inference();
    auto inferEnd = std::chrono::high_resolution_clock::now();
    auto inferTime = std::chrono::duration_cast<std::chrono::milliseconds>(inferEnd - inferStart).count();
    resProcess();
    publishMsg();
    auto end = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    #ifdef DEBUG
        // Draw reference point
        cv::circle(image, 
            refPoint, 
            5, cv::Scalar(0, 200, 200), -1);

        std::vector<std::string> hints = {
            "Side Dist: "+std::to_string(distance_side),
            "TrackStatus: "+std::string(TrackStatusHint[trackStatus_side]),
            "Lane Dist: "+std::to_string(distance_lane),
            "TrackStatus: "+std::string(TrackStatusHint[trackStatus_lane]),
            "InferTime: "+std::to_string(inferTime)+"ms",
            "TotalTime: "+std::to_string(totalTime)+"ms",
        };
        for(size_t each=0; each<hints.size(); each++){
            cv::putText(image, //target image
                hints[each],
                cv::Point(10, 35+40*each), //top-left position
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(200, 12, 200), //font color
                2);
        }
        #ifdef ALLOW_SHOW_CV
            if(show_cv){
                cv::imshow("image", image);
                cv::waitKey(1);
            }
        #endif
        publishImg();
    #endif
}