#include "PSDetLB.h"

PSDetectLB::PSDetectLB(){
    getLaunchParams();
    transaction();
    loadEngine();
}

void PSDetectLB::transaction(){
    PSDinfo_pub = nh.advertise<std_msgs::String>("/PSDetLB/info", 1);
    ROS_INFO("Subscribing to sensor image: %s", rImgName.c_str());

    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh));
    imageNode = it_->advertise("/PSDetLB/detImg", 1);
    sensorNode = it_->subscribe(rImgName, 
                                1000, 
                                &PSDetectLB::imageCallback, 
                                this);
    deduction = Deduction();
}

void PSDetectLB::getLaunchParams(){
    ros::param::get(ros::this_node::getName()+"/image_width", img_w);
    ros::param::get(ros::this_node::getName()+"/image_height", img_h);
    ros::param::get(ros::this_node::getName()+"/engine_name", engine_name);
    ros::param::get(ros::this_node::getName()+"/raw_image_node", rImgName);
    ROS_INFO("image: width => %d height => %d", img_w, img_h);
}

cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h);
void PSDetectLB::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    float data[3 * img_h * img_w];
    float prob[OUTPUT_SIZE];
    std::vector<Yolo::Detection> result_res;

    cv::Rect rect = cv::Rect(0, 0, 1280, 480);
    cv_ptr->image = cv::Mat(cv_ptr->image, rect);

    cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_90_CLOCKWISE);
    cv_ptr->image = inference(cv_ptr->image,*context, stream, buffers, data, prob, result_res);
    deduction.convert(&cv_ptr->image, result_res);
    deduction.visualization();

    // cv_ptr->image = preprocess_img(cv_ptr->image, img_w, img_h);
    imageNode.publish(cv_ptr->toImageMsg());
}



cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h) {
    int w, h, x, y;
    float r_w = input_w / (img.cols*1.0);
    float r_h = input_h / (img.rows*1.0);
    if (r_h > r_w) {
        w = input_w;
        h = r_w * img.rows;
        x = 0;
        y = (input_h - h) / 2;
    } else {
        w = r_h * img.cols;
        h = input_h;
        x = (input_w - w) / 2;
        y = 0;
    }
    cv::Mat re(h, w, CV_8UC3);
    cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(input_h, input_w, CV_8UC3, cv::Scalar(128, 128, 128));
    re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
    return out;
}

cv::Mat PSDetectLB::inference(cv::Mat img, 
                            IExecutionContext& context, 
                            cudaStream_t &stream,void** buffers, 
                            float* data, 
                            float* prob, 
                            std::vector<Yolo::Detection> &result_res){
    if (img.empty()) return img;
    cv::Mat pr_img = preprocess_img(img, img_w, img_h); // letterbox BGR to RGB
    int i = 0;
    for (int row = 0; row < img_h; ++row) {
        uchar* uc_pixel = pr_img.data + row * pr_img.step;
        for (int col = 0; col < img_w; ++col) {
            data[i] = (float)uc_pixel[2] / 255.0;
            data[i + img_h * img_w] = (float)uc_pixel[1] / 255.0;
            data[i + 2 * img_h * img_w] = (float)uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }

    // Run inference
    auto start = std::chrono::system_clock::now();
    int inputSize = 3 * img_h * img_w * sizeof(float);
    doInference(context, stream, buffers, data, inputSize, prob, OUTPUT_SIZE);
    auto end = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::vector<std::vector<Yolo::Detection>> batch_res(1);
    auto& res = batch_res[0];
    nms(res, &prob[0], CONF_THRESH, NMS_THRESH);
    cv::RNG rng(12345);
    cv::Scalar color[200];
    for(int i = 0;i<CLS_NUM;i++)
    {
         color[i] = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }
    #ifdef DRAW_ORIGIN_DET
        for (size_t j = 0; j < res.size(); j++) {
            cv::Rect r = get_rect(img, res[j].bbox);
            cv::rectangle(img, r, color[(int)res[j].class_id], 2);
            cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
    #endif
    result_res = res;
    return img;
}




void PSDetectLB::loadEngine(){
    ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    static Logger gLogger;
    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    //assert(engine->getNbBindings() == 2);
    //void* buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex("data");
    const int outputIndex = engine->getBindingIndex("prob");
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], 
                        3 * img_w * img_h * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], 
                        OUTPUT_SIZE * sizeof(float)));
    // Create stream
    CUDA_CHECK(cudaStreamCreate(&stream));
}

PSDetectLB::~PSDetectLB(){

}
