#ifndef CARCONTROL_H
#define CARCONTROL_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <math.h>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <ros/package.h>

#include <unistd.h>

#include "lanedetector.h"
#include "signrecognizer.h"

using namespace std;
using namespace cv;

class CarController
{
public:
    CarController(string team_name);
    ~CarController();

    int is_driving = 1;

    string team_name;

    LaneDetector *lane_detector;
    SignRecognizer *left_sign_recognizer;
    SignRecognizer *right_sign_recognizer;

    // fix size of input image
    int w;
    int h;

    int t=0;

    VideoWriter *video;

    int sign;

    double cur_speed; // current speed of the car

    void driverCar(float _angle, float velocity);

    /* Main Processing here
    - Receive image
    - Lane detecting
    - Sign Recognizing
    - Calculation Method
    - Driving Car
    */
    void main_processing(const Mat &img);

private:
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;

    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;
};

#endif
