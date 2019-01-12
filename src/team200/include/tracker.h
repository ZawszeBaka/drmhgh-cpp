#ifndef TRACKER_H
#define TRACKER_H

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

class Tracker
{
public:
    Tracker();
    ~Tracker();

    int minpix = 20;

    // bool isfirstframe = true;
    Rect2d up_track_win;
    Rect2d low_track_win;
    void init_tracker(const Mat &binary_warped,
                      Rect2d &up_track_win, // update up_track_win
                      Rect2d &low_track_win); // update low_track_win
    void tracking(const Mat &binary_warped, // input
                  Point2f &center_pts, // return
                  Mat &out_img); // update
    void find_center_pts(const Mat &binary_warped, // input
                         Point2f &center_pts, // return
                         Rect2d &up_track_win, // update
                         Rect2d &low_track_win, // update
                         Mat &out_img); // update


     bool is_inside_box(int win_y_low, int win_y_high,
                      int win_x_low, int win_x_high,
                      Point p);

      Mat findNonzero(Mat img); //

private:

};

#endif
