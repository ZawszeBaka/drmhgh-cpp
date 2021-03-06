#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/tracking.hpp>
// #include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

// #include <python3.6m/Python.h>

#include <typeinfo>

using namespace std;
using namespace cv;

#include "tracker.h"

// catkin_make && roslaunch lane_detect lane_de

class LaneDetector
{
public:
    LaneDetector();
    ~LaneDetector();

    int iframe= 0;

    double cur_speed=0; // current speed of the car

    // tracker
    bool IS_TRACKING = true;
    Tracker *h_tracker;
    Tracker *v_tracker;

    int orr = 1;

    /*
    Width and height of input image , which is fixed size !
    */
    int w = 320;
    int h = 240;

    VideoWriter *video;
    VideoWriter *video2;

    //
    bool reduced = true;
    bool is_test = false;
    bool is_save_fit_lines = true;
    VideoWriter writer ;

    /*
    Args:
      img (3 channels) of size width w and heigh h

    Returns:
      angle: angle (in degree limited between (-50,50))
      speed: speed
    */
    void detect(const Mat &img, const Mat &gray_img,
                double &angle, double &speed);

    // =============== STAGE 1 ===================
    // 4 chosen pts
    array<Point2f,4> src;
    array<Point2f,4> dst;

    // warping
    Mat addPoints(const Mat &img, array<Point2f,4> pts);
    Mat addPoints(const Mat &img, Point2f &p);
    Mat warp(const Mat &source);

    // =============== STAGE 2 ===================
    void choosing_thresholds_manually(); // manual trackbar

    // absolute sobel threshold
    int sobel_kernel = 3;  // 3x3 sobel kernel matrix
    std::array<int,2> abs_sobel_thresh_range {{0,255}}; // default for abs_sobel_thresh
    Mat abs_sobel_thresh(const Mat &gray, char orient); // orient = 'x' or 'y'

    // magnitude threshold
    array<int,2> mag_thresh_range {{30,100}}; // {0,255}
    Mat mag_thresh(const Mat &gray);

    // direction threshold
    array<int,2> tmp_dir_thresh_range{{22,41}};
    array<float,2> dir_thresh_range {{0.7, 1.3}}; // {0, M_PI/2}
    Mat dir_thresh(const Mat &gray);

    // apply gradient threshold: combination of
    //    absolute sobel threshold,
    //    magnitude threshold
    //    direction threshold
    array<int,2> abs_sobel_thresh_range_x {{20,100}};
    array<int,2> abs_sobel_thresh_range_y {{20,100}};
    Mat apply_gradient_threshold(const Mat &gray);

    // apply color threshold
    float s_thresh_min = 0 ;
    float s_thresh_max = 1 ;
    vector<int> color_thresh_low {102,63,63};
    vector<int> color_thresh_high {255,170,170};
    Mat apply_color_threshold(const Mat &img);

    // combine both gradient threshold and color threshold with operator OR
    Mat combine_threshold(const Mat &bi_grad, const Mat &bi_color);

    // ============== STAGE 3 ==================
    Mat get_histogram(const Mat &binary_warped);
    Mat get_histogram_v1(const Mat &binary_warped);

    // slide windows
    int margin = 40 ; // => window_width = 60
    int window_height = 30;
    int minpix = 20 ;

    int nwindows;
    int stridepix;
    int bwindow;
    int ewindow;

    // Mode "non-sign"
    // int non_nwindows = 15; // ewindow must be smaller than nwindows
    int non_stridepix = 15;
    int non_bwindow = 4;
    int non_ewindow = 6; //

    // Mode "left-sign" or "right-sign"
    // int sign_nwindows = 40; // ewindow must be smaller than nwindows
    // int sign_stridepix = 5;
    // int sign_bwindow = 10;
    // int sign_ewindow = 16;
    // int sign_nwindows = 15; // ewindow must be smaller than nwindows
    int sign_stridepix = 15;
    int sign_bwindow = 4;
    int sign_ewindow = 6;

    // Mode "horizontal"
    // int h_nwindows = 15; // ewindow must be smaller than nwindows
    int h_stridepix = 15;
    int h_bwindow = 1;
    int h_ewindow = 4;

    // return false if cannot detect lane
    bool slide_window(const Mat &binary_warped,
                      const Mat &v_histogram,
                      vector<Point2f> &v_center_windows,
                    vector<Point2f> &left_pts,
                  vector<Point2f> &right_pts,
                Mat &outimg);

    // sliding horizontal windows
    bool slide_window_v1(const Mat &binary_warped,
                      const Mat &h_histogram,
                      vector<Point2f> &h_center_windows,
                    vector<Point2f> &low_pts,
                  vector<Point2f> &up_pts,
                Mat &outimg);
    bool slide_window_v1(const Mat &binary_warped,
                        const Mat &h_histogram,
                        Rect2d &up,
                        Rect2d &low,
                        Mat &outimg);
    // sliding 3 lanes (vertical)
    bool slide_window_v3(const Mat &binary_warped,
                        const Mat &histogram,
                        vector<Point2f> &v_center_windows,
                        vector<Point2f> &left_pts,
                        vector<Point2f> &right_pts,
                        vector<Point2f> &mid_pts,
                        bool &left_status,
                        bool &right_status,
                        bool &mid_status,
                        Rect2d &left_rect,
                        Rect2d &right_rect,
                        Mat &out_img);
    Rect2d calc_rect(vector<Point2f> v, int w, int h);
    int find_midpoint(const Mat &hist, float eps);
    int find_midpoint_v1(const Mat &hist, float eps);
    double calc_mean(const Mat &hist, int x_min, int x_max);
    void calc_center_windows(vector<Point2f> &v_center_windows,
        vector<Point2f> &left_pts,
        vector<Point2f> &right_pts);

    // ============== STAGE 4 =================
    /*
      mode = 0  # left
      mode = 1  # right
      mode = 2  # non-sign
    */
    int mode = 2; // left, right non-sign
    bool left_flag=true; // !!
    bool right_flag=true;

    double distance;
    double marg;

    int turn_speed = 30; // speed when turning
    int normal_speed = 35; // speed when driving

    // just for trackbar
    static void change_turn_speed(int,void*);
    int tmp_turn_speed = 30;
    int tmp_normal_speed = 50;

    /*
      0 : non-sign
          (carcontroller)
      1 : waiting for speed down
          (calc_speed)  (set countdown = 2 )
                        (set left_flag and right_flag)
      2 : turning and countdown
          (calc_angle) (check countdown <= 0)
                      (mode = 2)
                      (set left_flag = true , right_flag= true)
    */
    int turn_state=0;
    void switchto0(bool);
    void switchto1(int sign);
    void switchto2();
    void switchto3();
    void switchto10();
    int countdown;
    int MAX_COUNTDOWN = 20;
    int countdown_mistaken = 0;
    int MAX_COUNTDOWN_MISTAKEN = 80;
    void mistaken();
    int tracking_countdown;
    int MAX_TRACKING_COUNTDOWN = 20;
    void keep_tracking();

    double angle_c, angle_s;

    vector<double>RANGE_ANGLE_SWITCH_TURN {{-35,35}}; // -40,40
    vector<double>RANGE_COUNTDOWN_ANGLE {{-5,5}}; // 15,15 if angle is between -30,30, countdown --
    vector<double>RANGE_ANGLE {{-50,50}}; // range of valid angle value
    double calc_angle(vector<Point2f> &center_windows,
                    vector<Point2f> &left_pts,
                  vector<Point2f> &right_pts,
                Mat &out_img);
    // calculate only 1 center pts
    double calc_angle(Point2f &center_pts, Mat &out_img);
    // calculate from multiple center windows
    double calc_angle(vector<Point2f> &center_windows,
                Mat &out_img);
    double calc_speed(vector<Point2f> &center_windows,
                    vector<Point2f> &left_pts,
                  vector<Point2f> &right_pts);



private:

    // Helper functions
    double find_max(Mat img);
    double find_min(Mat img);
    double find_maxx(vector<int> xs);
    double find_minx(vector<int> xs);
    void show_min_max(string name ,Mat img);
    double meanxy_of_pts(vector<Point2f> &pts, char axis);
    Point arg_max(Mat img);
    Mat sum(Mat img, char axis); // axis = 'x' or 'y'
    Mat plot_histogram(Mat hist);
    Mat dstack(const Mat &img);
    Mat findNonzero(Mat img); // Mat [Point, Point,...]
    void show_mat_type(string name ,Mat &img);
    string get_mat_type(Mat &img);
    bool is_inside_box(int win_y_low, int win_y_high,
                      int win_x_low, int win_x_high,
                      Point p);
    vector<double> polyfit(vector<int> vecX, vector<int> vecY, int nDegree); // https://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/
    Mat drawPolylines(Mat &img, vector<Point> pts);
    vector<Point> polyval(Mat &img, vector<double> coefs); // x = g(y)
    vector<Point> polyval(Mat &img, vector<double> coefs, bool is_pol);

    void show_histogram_normal(const Mat &img);
    void plot_binary_img(string name, const Mat &img);
    void plot_binary_img(string name, const Mat &img, int wait_time);

    void show_img_description(string name, const Mat &img);
    void show_mat_per(string name, const Mat &img, char dir); // dir = 'row' or 'col'

    void addText(Mat &img, string text, Point2f pos);
    string bool2str(bool b);


public:
    void videoProcess(string video_path);

};



#endif
