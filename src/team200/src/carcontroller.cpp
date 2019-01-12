#include "carcontroller.h"

CarController::CarController(string team_name)
{
    this->team_name = team_name;

    steer_publisher = node_obj1.advertise<std_msgs::Float32>(team_name+"_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>(team_name+"_speed",10);

    lane_detector = new LaneDetector();
    // left_sign_recognizer = new SignRecognizer("/home/non/Documents/ROS/drmhgh-cpp/src/team200/cascade/left.xml");
    // right_sign_recognizer = new SignRecognizer("/home/non/Documents/ROS/drmhgh-cpp/src/team200/cascade/right.xml");
    left_sign_recognizer = new SignRecognizer(ros::package::getPath("team200") + "/cascade/left.xml");
    left_sign_recognizer->s = 0;
    right_sign_recognizer = new SignRecognizer(ros::package::getPath("team200") + "/cascade/right.xml");
    right_sign_recognizer->s = 1;

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
    lane_detector->abs_sobel_thresh_range_x = {19,252};
    lane_detector->abs_sobel_thresh_range_y = {20,222};
    lane_detector->mag_thresh_range = {30,100};
    lane_detector->dir_thresh_range = {0.7,1.3};
    lane_detector->color_thresh_low = {102,63,63}; // bgr
    lane_detector->color_thresh_high = {255,170,170};
    // lane_detector->color_thresh_high = {170,170,170};

    // lane_detector->color_thresh_low = {180,180,200}; // bgr
    // lane_detector->color_thresh_high = {255,255,255};

    // STAGE 3 :
    // lane_detector->nwindows = lane_detector->non_nwindows;
    // lane_detector->stridepix = lane_detector->non_stridepix;
    // lane_detector->bwindow = lane_detector->non_bwindow;
    // lane_detector->ewindow = lane_detector->non_ewindow;

    // left_sign_recognizer->threshold_freq = 2;
    // right_sign_recognizer->threshold_freq = 2;

    cvCreateTrackbar("Left freq count", "Threshold", &(left_sign_recognizer->threshold_freq), 30);
    cvCreateTrackbar("right freq count", "Threshold", &(right_sign_recognizer->threshold_freq), 30);

    cvCreateTrackbar("Driving ?", "Threshold", &(is_driving), 1);

    // video = new VideoWriter("/home/non/Documents/Video/direction.avi",
    //                           CV_FOURCC('M','J','P','G'),10, Size(w,h));

}

CarController::~CarController()
{
    // video->release();
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

    Mat gray;
    cvtColor(img, gray, COLOR_RGB2GRAY);

    // lane detection
    lane_detector->detect(img, gray, angle, speed);

    Mat img_with_signs;

    t++; // make sure t is 0 and 1
    t = t%2;
    if(t==0){
      // sign recognition : move to the turn state 1
      left_sign_recognizer->iframe++;
      right_sign_recognizer->iframe++;
      if(lane_detector->turn_state == 0 ||
         lane_detector->turn_state == 10){ // non-sign
          sign = left_sign_recognizer->detect(img,gray,img_with_signs);
          if (sign!=2){
              lane_detector->switchto1(sign);
          }else{
            sign = right_sign_recognizer->detect(img,gray,img_with_signs);
            if (sign!=2){
              lane_detector->switchto1(sign);
            }
          }
      }
      // video->write(img_with_signs);
    }

    // cout << "[] color high " << lane_detector->color_thresh_high[0] << "\n";

    if(is_driving==1) driverCar(angle, speed);
    // cv::waitKey(1);
}
