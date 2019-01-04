#include "carcontroller.h"

CarController::CarController(string team_name)
{
    this->team_name = team_name;

    steer_publisher = node_obj1.advertise<std_msgs::Float32>(team_name+"_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>(team_name+"_speed",10);

    lane_detector = new LaneDetector();
    sign_recognizer = new SignRecognizer();

    w = lane_detector->w;
    h = lane_detector->h;

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



}

CarController::~CarController()
{
    cv::destroyAllWindows();
}

void CarController::driverCar(float _angle, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = _angle;
    speed.data = velocity;

    steer_publisher.publish(angle);
    usleep(1);
    speed_publisher.publish(speed);
    usleep(1);

    // std::cout << " [INFO] Driving: angle = " << angle.data << ", speed = " << speed.data << std::endl;
}

void CarController::main_processing(const Mat &img)
{
    /* Main Processing here
    - Receive image
    - Lane detecting
    - Sign Recognizing
    - Calculation Method
    - Driving Car
    */
    double angle;
    double speed;
    lane_detector->detect(img, angle, speed);
    sign_recognizer->detect(img);
    driverCar(angle, speed);
    cv::waitKey(1);
}
