// for getting current directory execution
#include <string>
#include <limits.h>
#include <unistd.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ctime>
#include <boost/timer.hpp>

#include <ros/package.h> 

#include "helperfunctions.h"
#include "carcontroller.h"
#include "svmprocess.h"

using namespace std;
using namespace cv;

// ------------------------------------
string team_name = "Team1";
// ------------------------------------

CarController *carcontroller;
SVMProcess *svmprocess;

string getexepath()
{
    // for getting current directory execution
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    return std::string( result, (count > 0) ? count : 0 );
}

// check for connection
bool CHECK_CON = false;

// Compute Fps
int check_frames = 100;
int iframe = 0;
boost::timer t;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // When subscribing  images
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

        // put img for carcontroller
        carcontroller->main_processing(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout<< " [Error detected] !" << std::endl;
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


/* MAIN */
int main(int argc, char **argv)
{
    // Initialize and Activate
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::startWindowThread();

    // CarController is the main processing including
    // lane detecting, sign recognizer and
    // computing angle,speed for drving car
    carcontroller = new CarController(team_name);

    // SHOW INFORMATION
    cout << "[INFO] Team name: " << team_name << "\n";
    cout << "[INFO] Image Size = " << Size(carcontroller->h,carcontroller->w) << "\n";
    cout << "[INFO] OpenCV version : " << CV_VERSION << " , " << CV_MAJOR_VERSION << "\n";
    cout << "[INFO] Current dir: " << getexepath() << "\n";
    cout << "[INFO] Package directory " << ros::package::getPath("lane_detect") << "\n";

    // for train SVM
    svmprocess = new SVMProcess();


    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(team_name + "_image", 1, image_callback);

    // Single-threaded Spinning
    ros::spin();
}
