#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

#include <python3.6m/Python.h>

#include <typeinfo>

using namespace std;
using namespace cv;

// catkin_make && roslaunch lane_detect lane_de

class LaneDetector
{
public:
    LaneDetector();
    ~LaneDetector();

    /*
    Width and height of input image , which is fixed size !
    */
    int w = 320;
    int h = 240;

    /*

    */
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
    void detect(const Mat &img, double &angle, double &speed);

    // =============== STAGE 1 ===================
    // 4 chosen pts
    array<Point2f,4> src;
    array<Point2f,4> dst;

    // warping
    Mat addPoints(const Mat &img, array<Point2f,4> pts);
    Mat addPoints(const Mat &img, vector<int> x, vector<int> y);
    Mat warp(const Mat &source);

    // =============== STAGE 2 ===================
    void choosing_thresholds_manually(); // manual trackbar

    // absolute sobel threshold
    int sobel_kernel = 3;  // 3x3 sobel kernel matrix
    array<int,2> abs_sobel_thresh_range = {0,255}; // default for abs_sobel_thresh
    Mat abs_sobel_thresh(const Mat &gray, char orient); // orient = 'x' or 'y'

    // magnitude threshold
    array<int,2> mag_thresh_range = {30,100}; // {0,255}
    Mat mag_thresh(const Mat &gray);

    // direction threshold
    array<float,2> dir_thresh_range = {0.7, 1.3}; // {0, M_PI/2}
    Mat dir_thresh(const Mat &gray);

    // apply gradient threshold: combination of
    //    absolute sobel threshold,
    //    magnitude threshold
    //    direction threshold
    array<int,2> abs_sobel_thresh_range_x = {20,100};
    array<int,2> abs_sobel_thresh_range_y = {20,100};
    Mat apply_gradient_threshold(const Mat &gray);

    // apply color threshold
    float s_thresh_min = 0 ;
    float s_thresh_max = 1 ;
    vector<double> color_thresh_low = {0,0,0};
    vector<double> color_thresh_high = {10,10,10};
    Mat apply_color_threshold(const Mat &img);

    // combine both gradient threshold and color threshold with operator OR
    Mat combine_threshold(const Mat &bi_grad, const Mat &bi_color);

    // ============== STAGE 3 ==================
    Mat get_histogram(const Mat &binary_warped);

    // slide windows
    int margin = 50 ;
    int minpix = 50 ;
    int nwindows = 9;
    int n_dim = 2;
    // return false if not have fit line
    bool slide_window(const Mat &binary_warped,
                      const Mat &histogram,
                      vector<Point> &out_left_plot,
                      vector<Point> &out_right_plot ,
                      vector<double> &out_left_fit,
                      vector<double> &out_right_fit);
    void test_slide_window(const Mat &img);


    // ============== STAGE 4 =================

    // draw detected lines in image
    Mat original_image_with_lines(Mat &img,
                                  vector<Point> left_pts,
                                  vector<Point> right_pts);
    double finding_angle_direction(Mat binary_img,
                                   vector<double> &left_coefs,
                                   vector<double> &right_coefs);
    double finding_angle_direction(Mat binary_img);


    Point2f upper_center;
    Point2f lower_center;



private:

    // Helper functions
    double find_max(Mat img);
    double find_min(Mat img);
    void show_min_max(string name ,Mat img);
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

public:
    void videoProcess(string video_path);


    // vector<Point> leftLane, rightLane;
    // Mat preProcess(const Mat &src);
    //
    // Mat morphological(const Mat &imgHSV);
    //
    // void fillLane(Mat &src);
    // vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    // vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    // void detectLeftRight(const vector<vector<Point> > &points);
    // Mat laneInShadow(const Mat &src);
    //
    // // Hue, Saturation, Value thresholds
    // int minThreshold[3] = {0, 0, 180};
    // int maxThreshold[3] = {179, 30, 255};
    //
    // int minShadowTh[3] = {90, 43, 36};
    // int maxShadowTh[3] = {120, 81, 171};
    // int minLaneInShadow[3] = {90, 43, 97};
    // int maxLaneInShadow[3] = {120, 80, 171};
    // int binaryThreshold = 180;
    //
    // int skyLine = 85;
    // int shadowParam = 40;

};



#endif
