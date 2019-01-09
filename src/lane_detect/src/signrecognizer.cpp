#include "signrecognizer.h"

SignRecognizer::SignRecognizer(string filepath)
{
    // if using Haar Cascade Detection
    bool ret = sign_cascade.load(filepath);
    if (!ret){
      // std::cout << "[ERROR] Reading cascade file Failed ! Make sure the path exists \n";
    }
    svmprocess = new SVMProcess();
}

SignRecognizer::~SignRecognizer()
{

}

int SignRecognizer::detect(const Mat &img, const Mat &gray_img)
{
    // Rect(x,y,width,height)
    Rect haft_top(0,0,img.size().width,(int)img.size().height/2);
    Mat gray(gray_img);

    Rect s;
    Mat tmp_region_of_sign;
    Mat region_of_sign;

    Mat img_with_signs(img);

    // // Using haar cascade detection
    bool is_detected = haarcascade_detect(img, gray, s, img_with_signs);
    //
    // draw rectangle if detecting signs
    if (is_detected){
        tmp_region_of_sign = img(s);
        cv::resize(tmp_region_of_sign, region_of_sign,Size(20,20),0,0, cv::INTER_LINEAR );
        int rs = svmprocess->predict(region_of_sign);
        // cout << "[DEBUG] Detected region ! : " << rs << "\n";
        // region_of_sign = tmp_region_of_sign;
        // img_with_signs(Rect(0,0,region_of_sign.size().width,region_of_sign.size().height)) = region_of_sign;
        if(rs == 0) {freq_left++;freq_right--;} // left
        else if(rs == 1){freq_left--;freq_right++;} // right
        // else {freq_left--;freq_right--;}
        if((freq_left>=threshold_freq) || (freq_right>=threshold_freq))
            return rs;
    } else {
        if (freq_left > 0) freq_left--;
        if (freq_right > 0) freq_right--;
    }

    return 2; // non-sign

    // cv::imshow("Sign Detection", img_with_signs);

}

bool SignRecognizer::haarcascade_detect(const Mat &img,
        const Mat &gray, Rect &s, Mat &img_with_signs)
{
    vector<Rect> signs;
    /*
    gray image
    vector<Rect> &signs
    double scaleFactor : specifying how much the image size is reduced at each image scale
    int minNeighbors : specifying how many neighbors each candidate rectangle should have to retain it
    int flags
    Size minSize: minimum possible object size
    Size maxSize: maximum possible object size
    */
    // Mat tmp;
    // resize(gray,tmp,Size(60,60));
    sign_cascade.detectMultiScale(gray, signs, 1.1, 3, INTER_LINEAR, Size(20,20), gray.size());

    int num_detected_signs = (int) signs.size();

    if (num_detected_signs == 0){
         return false;
    }

    if (num_detected_signs >= 1){
        int max_size = 0;
        for (int i = 0; i < num_detected_signs; i++){
            int w = signs[i].width;
            int h = signs[i].height;
            if (w*h > max_size){
                s = signs[i];
            }
            rectangle(img_with_signs, signs[i], Scalar(0,255,0));
        }
        return true;
    }
}
