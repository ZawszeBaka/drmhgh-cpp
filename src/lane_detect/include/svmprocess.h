#ifndef SVM_H
#define SVM_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include <fstream>
#include <string>

#include <ros/ros.h>

using namespace std;
using namespace cv;
using namespace cv::ml;

class SVMProcess
{
public:
    SVMProcess();
    ~SVMProcess();

    Ptr<SVM> svm ;

    Mat X; // data
    Mat y; // labels

    void get_train_data(string path);
    void train(const Mat &X, const Mat &y);
    void predict();

private:

};
#endif