#include "svmprocess.h"

SVMProcess::SVMProcess()
{
    svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));

    /*
      Size winSize
      Size blockSize
      Size blockStride
      Size cellSize
      int nbins
    */
    winSize = new Size(20,20);
    blockSize = new Size(4,4);
    blockStride = new Size(2,2);
    cellSize = new Size(2,2);
    nbins = 9;
    hog = new HOGDescriptor(*winSize, *blockSize, *blockStride, *cellSize, nbins);

    this->get_train_data();
}

SVMProcess::~SVMProcess()
{

}

void SVMProcess::get_train_data()
{
  	// read description file
  	ifstream f("/home/non/Documents/data/des.txt");
    vector<Mat> imgs;
    vector<vector<float>> data;
    string str;
  	while(getline(f,str))
  	{
      string fullpath(str, 1, str.length()-1);
      fullpath = "/home/non/Documents/data" + fullpath;
      Mat rimg = imread(fullpath, IMREAD_GRAYSCALE);
      Mat img;
      resize(rimg,img, *winSize);
      imshow("test",img);
      waitKey();

      data.push_back(extract_HOG(img));
      imgs.push_back(img);
  	}
    f.close();
}

vector<float> SVMProcess::extract_HOG(const Mat &gray)
{
    vector<float> desc;
    vector<Point> locations;
    hog->compute(gray,desc,Size(0,0),Size(0,0),locations);

    cout << "[INFO] Amount of HOG descriptor : " << desc.size() << "\n";

    return desc;
}

void SVMProcess::train(const Mat &X, const Mat &y)
{
	/*
		Input:
			X: training data
			y: training label

		int labels[4] = {1, -1, -1, -1};
	    float trainingData[4][2] = { {501, 10}, {255, 10}, {501, 255}, {10, 501} };
	    Mat X(4, 2, CV_32F, trainingData);
	    Mat y(4, 1, CV_32SC1, labels);

	*/
	svm->train(X, ROW_SAMPLE, y);
}

void SVMProcess::save_model(string path)
{
	svm->save(path);
}

void SVMProcess::load_model(string path)
{
	svm->load(path);
}

void SVMProcess::predict()
{

}
