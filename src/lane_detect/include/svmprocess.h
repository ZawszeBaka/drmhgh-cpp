#ifndef SVM_H
#define SVM_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/objdetect.hpp>

#include <fstream>

#include <ros/package.h>
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
    Ptr<HOGDescriptor> hog;

    Mat *X;
    Mat *y;

    vector<string> desc_labels {{"left-sign","right-sign","non-sign"}};

    void get_train_data(Size winSize);
    Mat extract_HOG(const Mat &gray);

    void train(const Mat &X, const Mat &y);
    void train();

    string predict(const Mat &gray);
    void save_model(string path);
    void load_model(string path);

    // --- Helper functions
    Mat cvtVecOfVec2Mat(vector<vector<float>> angles);
private:
};
#endif
