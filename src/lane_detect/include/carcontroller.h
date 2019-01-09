#ifndef CARCONTROL_H
#define CARCONTROL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <math.h>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

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

    string team_name;

    LaneDetector *lane_detector;
    SignRecognizer *sign_recognizer;

    // fix size of input image
    int w;
    int h;


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
