#ifndef SVM_H
#define SVM_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>

#include "signrecognizer.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

class SVMProcess
{
public:
    SVMProcess();
    ~SVMProcess();

    Ptr<SVM> svm ;
    Ptr<HOGDescriptor> hog;

    Mat* X; // data
    Mat* y; // labels

    Size *winSize;
    Size *blockSize;
    Size *blockStride;
    Size *cellSize;
    int nbins;

    void get_train_data();
    vector<float> extract_HOG(const Mat &gray);

    void train(const Mat &X, const Mat &y);
    void predict();
    void save_model(string path);
    void load_model(string path);
private:

};
#endif
