#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"


DetectLane *detect;
CarControl *car;
int skipFrame = 1;

auto samplePic = cv::imread("/home/yus/Documents/Pic/abc.png");
VideoWriter writer;

double angle , speed ;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        out = cv_bridge::toCvShare(msg, "bgr8")->image;

        // cv::imshow("View", out);
        // cv::waitKey(30);

        // cv::imwrite("/home/non/Documents/Pic/view.png", out);
        writer << out ;

        detect->update(cv_ptr->image, angle, speed);
        car->driverCar(angle, speed);

        // detect->update(cv_ptr->image);
        // car->driverCar(detect->getLeftLane(), detect->getRightLane(), 50);

    }
    catch (cv_bridge::Exception& e)
    {
        std::cout<< " [Error detected] !" << std::endl;
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    // initialize ~
    detect = new DetectLane();
    car = new CarControl();

    int width = samplePic.size().width;
    int height = samplePic.size().height;


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
    detect->src = {
      Point2f(width*2/7-10,height/2-10), // top left
      Point2f(0,height*4/5-3), // bottom left
      Point2f(width*6/7+35,height*4/5-3), // top right
      Point2f(width*2/3+10,height/2-10) // bottom right
    };

    detect->dst = {
      Point2f(width*2/7-10,height/2-10),
      Point2f(width*2/7-10,height*4/5-3),
      Point2f(width*5/7+10,height*4/5-3),
      Point2f(width*5/7+10,height/2-10)
    };

    // STAGE 2 :
    detect->sobel_kernel = 3;
    detect->abs_sobel_thresh_range_x = {20,100};
    detect->abs_sobel_thresh_range_y = {20,100};
    detect->mag_thresh_range = {30,100};
    detect->dir_thresh_range = {0.7,1.3};
    detect->color_thresh_low = {102,63,63}; // bgr
    detect->color_thresh_high = {255,170,170};
    // detect->color_thresh_low = {180,180,200}; // bgr
    // detect->color_thresh_high = {255,255,255};

    // STAGE 3 :
    detect->margin =40;
    detect->minpix = 20;
    detect->nwindows = 9;
    detect->n_dim = 2; //  = 1 linear regression  , = 2 square function


    // STAGE 3:


    // ========================<>=======================
    // Team1 - ws://127.0.0.1:9090

    if (is_test_image) {

        detect->is_test = true ;
        Mat img = cv::imread("/home/yus/Documents/Pic/noise.png");

        // detect->test_warp();
        // detect->test_apply_gradient_threshold();
        // detect->test_slide_window(img);

        double angle = 0;
        double spped = 0;
        detect->update(img, angle, speed );

        // choosing value manually
        // detect->choosing_thresholds_manually(img);

    } else if(is_test_video){

        // path file , the order of frame
        //220
        //250
        detect->videoProcess("/home/yus/Documents/Video/out.avi",
                             "/home/yus/Documents/Pic/noise11.png",
                             i_test_video);

    } else if(is_real){

        Mat img = cv::imread("/home/yus/Documents/Pic/noise.png");
        writer = cv::VideoWriter("/home/yus/Documents/Video/out.avi", VideoWriter::fourcc('M','J','P','G'), 24, samplePic.size()); // 24 Fps
        std::cout << " [INFO] size = " <<  samplePic.size() << std::endl;
        std::cout << "[INFO] OpenCV version : " << CV_VERSION << endl;
        ros::NodeHandle nh;
        // cv::namedWindow("View");
        // cv::startWindowThread();
        // imshow("View", img);
        // waitKey(20);
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

        ros::spin();
        writer.release();
    } else {

    }

    cv::destroyAllWindows();
}
