#include "svmprocess.h"

SVMProcess::SVMProcess()
{
    // svm = SVM::create();
    // svm->setType(SVM::C_SVC);
    // svm->setKernel(SVM::LINEAR);
    // svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));

    svm = Algorithm::load<SVM>("/home/non/Documents/data/train_svm/model/out.xml");

    /*
      Size winSize
      Size blockSize
      Size blockStride
      Size cellSize
      int nbins
    */
    Size winSize(20,20);
    Size blockSize(4,4);
    Size blockStride(2,2);
    Size cellSize(2,2);
    int nbins = 9;
    hog = new HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins);

}

SVMProcess::~SVMProcess()
{

}

Mat SVMProcess::extract_HOG(const Mat &gray)
{
    vector<float> desc;
    vector<Point> locations;
    hog->compute(gray,desc,Size(0,0),Size(0,0),locations);
    Mat p(1,(int)desc.size(),CV_32F);
    for(int i=0; i<p.cols; ++i) p.at<float>(0, i) = desc.at(i);
    return p;
}

string SVMProcess::predict(const Mat &gray)
{
    Mat hog_gray = extract_HOG(gray);
    float val = svm->predict(hog_gray);
    if ((val < desc_labels.size()) && (val >= 0))
    {
        return desc_labels[(int)val];
    }
    return "non-sign";
}
