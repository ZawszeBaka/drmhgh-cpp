#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "lanedetector.h"
#include "carcontroller.h"
#include "signrecognizer.h"

#include <ctime>
#include <boost/timer.hpp>

LaneDetector *lane_detector;
CarController *car;

double angle , speed ;

// Compute FPS
int check_frames = 100;
int iframe = 0;
boost::timer t;


bool CHECK_CON = false; // check for connection

// When subscribing  images
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Compute Fps
    if (iframe == 0){
        // begin
        // start = clock();
        t.restart();
        iframe++;
    }else if (iframe == check_frames){
        // float dif = (float)std::time(0) - (float)start;
        // std::cout << "[DEBUG] end " << (double)std::time(0) << " , start = " << (double)start << "\n";
        // float dur = 1/(dif)*check_frames;
        std::cout << "[INFO] " << 1/ ((float)t.elapsed()) *100 << " FPS \n";
        iframe = 0;
    }else{
        iframe++;
    }

    if (CHECK_CON){
        std::cout << "[INFO] Receiving image ! \n";
    }

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        out = cv_bridge::toCvShare(msg, "bgr8")->image;
        // writer << out ;
        imshow("View", out);
        waitKey(1);

        lane_detector->detect(cv_ptr->image, angle, speed);
        car->driverCar(angle, speed);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout<< " [Error detected] !" << std::endl;
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

/* MAIN PROCESS */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    // initialize ~
    lane_detector = new LaneDetector();
    car = new CarController();

    int w = lane_detector->w;
    int h = lane_detector->h;

    // ======================================================
    // ======================= SET VALUE ====================
    // ======================================================
    // Team1 - ws://127.0.0.1:9090
    /*
      It follows the priority ! if the previous one is set to true
      then, all the next ones cannot be executed
    */
    bool is_test_image = false ;
    bool is_test_video = false;
    bool is_real = true ;
    int i_test_video = 14*24;  // 1s = 24 frames

    // STAGE 1 : choosing 4 points
    lane_detector->src = {
      Point2f(w*2/7-10,h/2-10), // top left
      Point2f(0,h*4/5-3), // bottom left
      Point2f(w*6/7+35,h*4/5-3), // top right
      Point2f(w*2/3+10,h/2-10) // bottom right
    };

    lane_detector->dst = {
      Point2f(w*2/7-10,h/2-10),
      Point2f(w*2/7-10,h*4/5-3),
      Point2f(w*5/7+10,h*4/5-3),
      Point2f(w*5/7+10,h/2-10)
    };

    // STAGE 2 :
    lane_detector->sobel_kernel = 3;
    lane_detector->abs_sobel_thresh_range_x = {20,100};
    lane_detector->abs_sobel_thresh_range_y = {20,100};
    lane_detector->mag_thresh_range = {30,100};
    lane_detector->dir_thresh_range = {0.7,1.3};
    lane_detector->color_thresh_low = {102,63,63}; // bgr
    lane_detector->color_thresh_high = {255,170,170};
    // lane_detector->color_thresh_low = {180,180,200}; // bgr
    // lane_detector->color_thresh_high = {255,255,255};

    // STAGE 3 :
    lane_detector->margin =40;
    lane_detector->minpix = 20;
    lane_detector->nwindows = 9;
    lane_detector->ewindow = 4;

    // STAGE 3:


    // ========================<>=======================
    // Team1 - ws://127.0.0.1:9090

    if (is_test_image) {

        lane_detector->is_test = true ;
        Mat img = cv::imread("/home/yus/Documents/Pic/noise.png");

        double angle = 0;
        double spped = 0;
        lane_detector->detect(img, angle, speed );

    } else if(is_test_video){

        // path file , the order of frame
        //220
        //250
        lane_detector->videoProcess("/home/yus/Documents/Video/out.avi");

    } else if(is_real){

        cv::namedWindow("View");
        cv::startWindowThread();

        // writer = cv::VideoWriter("/home/yus/Documents/Video/out.avi", VideoWriter::fourcc('M','J','P','G'), 24, Size(h,w)); // 24 Fps
        std::cout << " [INFO] Image Size = " << Size(h,w) << std::endl;
        std::cout << "[INFO] OpenCV version : " << CV_VERSION << endl;
        ros::NodeHandle nh;

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

        ros::spin();
        // writer.release();
    } else {

    }

    cv::destroyAllWindows();
}
