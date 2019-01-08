#include "svmprocess.h"

SVMProcess::SVMProcess()
{
    svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));

}

SVMProcess::~SVMProcess()
{

}

void get_train_data()
{
	// read description file 
	std::ifstream file("/home/non/Documents/data/des.txt");
	std::string str;
	while(std::getline(file,str))
	{
		cout << "[INFO] " << str << "\n";
	}
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