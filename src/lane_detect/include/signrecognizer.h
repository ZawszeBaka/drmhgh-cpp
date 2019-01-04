#ifndef SIGNRECOGNIZER_H
#define SIGNRECOGNIZER_H

#include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/objdetect/detection_based_tracker.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>

// #include <ros/ros.h>

using namespace std;
using namespace cv;

class SignRecognizer
{
public:
    SignRecognizer();
    ~SignRecognizer();

    void detect(const Mat &gray);

    // if using haar cascade detection
    CascadeClassifier sign_cascade;
    bool haarcascade_detect(const Mat &img,
            const Mat &gray, Rect &s, Mat &img_with_signs);// returns true if detected at least 1 sign



private:

};

#endif
