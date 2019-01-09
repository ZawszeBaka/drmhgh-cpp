#include "lanedetector.h"

LaneDetector::LaneDetector() {
    // cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    // cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);
    //
    // cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    // cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);
    //
    // cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    // cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);
    //
    // cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
    video = new VideoWriter("/home/non/Documents/Video/bi.avi",CV_FOURCC('M','J','P','G'),10, Size(w,h));
    video2 = new VideoWriter("/home/non/Documents/Video/direction.avi",CV_FOURCC('M','J','P','G'),10, Size(w,h));
}

LaneDetector::~LaneDetector(){
    video->release();
    video2->release();
}

void LaneDetector::detect(const Mat &img, const Mat &gray_img,
                          double &angle, double &speed)
{
    // STAGE 2
    // gradient threshold

    // cout << "[DEBUG] mode = " << mode << " , ready = " << ready << ", countdown = " << countdown << ", speed = " << cur_speed << "\n";

    reduced = true;

    Mat gray(gray_img);
    // cvtColor(, gray, COLOR_RGB2GRAY);

    Mat bi_grad = apply_gradient_threshold(gray);

    // color threshold
    Mat bi_color = apply_color_threshold(img);

    // combine color and gradient thresholds !!
    Mat combined_binary = combine_threshold(bi_grad, bi_color);

    // warping
    Mat binary_warped = warp(combined_binary);

    // STAGE 3
    Mat histogram = get_histogram(binary_warped);

    vector<Point> left_plot, right_plot;
    vector<double> left_fit, right_fit;

    reduced = false;
    vector<Point2f> center_windows;
    vector<Point2f> left_pts;
    vector<Point2f> right_pts;
    bool ret = slide_window(binary_warped, histogram, center_windows, left_pts, right_pts);

    if (ret)
    {
        angle = calc_angle(center_windows,left_pts,right_pts,gray);
        speed = calc_speed(center_windows,left_pts,right_pts);
    }else{
        cout << "[INFO] Cannot find lane ! \n";
        // waitKey();
    }
}

/*====================STAGE 1=====================*/
// add circles at 4 points in image
Mat LaneDetector::addPoints(const Mat &img, const array<Point2f,4> pts)
{
    Mat img_with_4pts(img) ;
    // img.copyTo(img_with_4pts);
    for( size_t i = 0; i < pts.size(); i++ )
    {
        int x = round(pts.at(i).x);
        int y = round(pts.at(i).y);
        cv::rectangle(img_with_4pts, Point(x-1,y-1), Point(x+1,y+1),cv::Scalar(0,255,0),10);
    }
    return img_with_4pts;
}

Mat LaneDetector::addPoints(const Mat &img, vector<int> x, vector<int> y)
{
    Mat img_with_pts ;
    img.copyTo(img_with_pts);
    int sz = x.size();
    // for( int i = 0; i < sz; i++ )
    // {
    //     cv::rectangle(img_with_pts, Point(x[i]-1,y[i]-1), Point(x[i]+1,y[i]+1),cv::Scalar(0,255,0),5);
    // }
    return img_with_pts;
}

// warping
Mat LaneDetector::warp(const Mat &img)
{
    // convert to Point2f array
    Point2f src[4];
    for(int i = 0 ; i < 4; i++){
        src[i] = this->src.at(i);
    }
    Point2f dst[4];
    for(int i = 0 ; i < 4; i++){
        dst[i] = this->dst.at(i);
    }
    Mat M = getPerspectiveTransform(src, dst);
    Mat warped_img(img.rows, img.cols, CV_8UC3);
    warpPerspective(img, warped_img, M, warped_img.size(), INTER_LINEAR, BORDER_CONSTANT);

    if(!reduced){
        Mat tmp(img);
        if (get_mat_type(tmp) == "8UC1")
        {
            plot_binary_img(" Stage 2 - Warped Binaric Image" , warped_img);
        } else{
            imshow(" Stage 2 - Warped Image", warped_img);
            // waitKey(1);
        }
    }

    return warped_img;
}

/*====================STAGE 2=====================*/
// Manual Bar
void LaneDetector::choosing_thresholds_manually()
{

}

// absolute sobel threshold
Mat LaneDetector::abs_sobel_thresh(const Mat &gray, char orient)
{
    // applying sobel filter corresponding to axis x or y
    Mat sobel;
    if (orient == 'x'){
        // follow the x axis
        cv::Sobel(gray, sobel, CV_64F, 1, 0, sobel_kernel);
        // src, dst, ddepth, x-axis , y-axis, ksize
    } else {
        // follow the y axis
        cv::Sobel(gray, sobel, CV_64F, 0, 1, sobel_kernel);
        // src, dst, ddepth, x-axis, y-axis, ksize
    }

    // after applying kernel, value of each pixel can be exceed 255 or lower than 0
    // so we need to get the absolute value and scale it down to the range 0-255
    Mat abs_sobel = cv::abs(sobel);
    Mat scaled_sobel = abs_sobel * 255 / find_max(abs_sobel);

    Mat grad_binary_thresh_min, grad_binary_thresh_max ;

    // THRESH_BINARY
    // dst(x,y) = 1   if src(x,y) > thresh_min
    //            0   otherwise
    cv::threshold(scaled_sobel,grad_binary_thresh_min, abs_sobel_thresh_range.at(0), 1, cv::THRESH_BINARY );

    // THRESH_BINARY_INV
    // dst(x,y) = 0  if src(x,y) > thresh_max
    //            1  otherwise
    cv::threshold(scaled_sobel, grad_binary_thresh_max, abs_sobel_thresh_range.at(1), 1, cv::THRESH_BINARY_INV);

    // convert to binary
    grad_binary_thresh_min.convertTo(grad_binary_thresh_min, CV_8UC1, 1.0);
    grad_binary_thresh_max.convertTo(grad_binary_thresh_max, CV_8UC1, 1.0);

    Mat grad_binary = grad_binary_thresh_max & grad_binary_thresh_min;

    return grad_binary;
}

// magnitude threshold
Mat LaneDetector::mag_thresh(const Mat &gray)
{
    Mat sobelx, sobely;
    // applying sobel filter corresponding to axis x or y
    // follow the x axis
    cv::Sobel(gray, sobelx, CV_64F, 1, 0, sobel_kernel);
    // src, dst, ddepth, x-axis , y-axis, ksize

    // follow the y axis
    cv::Sobel(gray, sobely, CV_64F, 0, 1, sobel_kernel);
    // src, dst, ddepth, x-axis, y-axis, ksize

    //    = sqrt( sobelx ^ 2 + sobely ^ 2)
    Mat abs_sobel;
    cv::magnitude(sobelx, sobely, abs_sobel);

    // after applying kernel, value of each pixel can be exceed 255 or lower than 0
    // so we need to get the absolute value and scale it down to the range 0-255
    Mat scaled_sobel = abs_sobel * 255 / find_max(abs_sobel);

    Mat mag_binary_thresh_max, mag_binary_thresh_min;

    // THRESH_BINARY
    // dst(x,y) = 1   if src(x,y) > thresh_min
    //            0   otherwise
    cv::threshold(scaled_sobel,mag_binary_thresh_min, mag_thresh_range.at(0), 1, cv::THRESH_BINARY );

    // THRESH_BINARY_INV
    // dst(x,y) = 0  if src(x,y) > thresh_max
    //            1  otherwise
    cv::threshold(scaled_sobel, mag_binary_thresh_max, mag_thresh_range.at(1), 1, cv::THRESH_BINARY_INV);
    mag_binary_thresh_min.convertTo(mag_binary_thresh_min, CV_8UC1, 1.0);
    mag_binary_thresh_max.convertTo(mag_binary_thresh_max, CV_8UC1, 1.0);

    Mat mag_binary = mag_binary_thresh_min & mag_binary_thresh_max;

    return mag_binary;
}

Mat LaneDetector::dir_thresh(const Mat &gray)
{
    Mat sobelx, sobely;
    // applying sobel filter corresponding to axis x or y
    // follow the x axis
    cv::Sobel(gray, sobelx, CV_8UC1, 1, 0, sobel_kernel);
    // src, dst, ddepth, x-axis , y-axis, ksize

    // follow the y axis
    cv::Sobel(gray, sobely, CV_8UC1, 0, 1, sobel_kernel);
    // src, dst, ddepth, x-axis, y-axis, ksize

    Mat abs_sobelx = cv::abs(sobelx);
    Mat abs_sobely = cv::abs(sobely);

    int height = sobelx.rows;
    int width = sobelx.cols;

    Mat grad_dir = Mat::zeros(height,width, CV_64F);
    // arctan(y/x)
    for(int i = 0; i < height; i++)
    {
        for(int j = 0 ; j < width; j++)
        {
            double x = sobelx.at<uchar>(i,j);
            double y = sobely.at<uchar>(i,j);
            grad_dir.at<double>(i,j) = atan2(y,x);
        }
    }

    // an array of zeros with the same shape and type as a given array
    Mat dir_binary_thresh_min, dir_binary_thresh_max ;

    // THRESH_BINARY
    // dst(x,y) = 1   if src(x,y) > thresh_min
    //            0   otherwise
    cv::threshold(grad_dir,dir_binary_thresh_min, dir_thresh_range.at(0), 1, cv::THRESH_BINARY );

    // THRESH_BINARY_INV
    // dst(x,y) = 0  if src(x,y) > thresh_max
    //            1  otherwise
    cv::threshold(grad_dir, dir_binary_thresh_max, dir_thresh_range.at(1), 1, cv::THRESH_BINARY_INV);

    // convert to int 0 1
    dir_binary_thresh_min.convertTo(dir_binary_thresh_min, CV_8UC1, 1.0);
    dir_binary_thresh_max.convertTo(dir_binary_thresh_max, CV_8UC1, 1.0);

    Mat dir_binary = dir_binary_thresh_min & dir_binary_thresh_max;

    return dir_binary;
}

Mat LaneDetector::apply_gradient_threshold(const Mat &gray)
{
    // sobel  x-axis, y-axis
    abs_sobel_thresh_range = abs_sobel_thresh_range_x;
    Mat gradx = abs_sobel_thresh(gray, 'x');
    abs_sobel_thresh_range = abs_sobel_thresh_range_y;
    Mat grady = abs_sobel_thresh(gray, 'y');

    // magnitude
    Mat mag_binary = mag_thresh(gray);

    // direction (angle)
    Mat dir_binary = dir_thresh(gray);

    // combine magnitude and direction (thresholded)
    Mat bi_grad = (gradx & grady) | (mag_binary & dir_binary);

    return bi_grad ;
}

Mat LaneDetector::apply_color_threshold(const Mat &img)
{
    Mat rs(img);
    GaussianBlur(img,img,Size(3,3),2);
    inRange(img, color_thresh_low, color_thresh_high, rs);

    // scale to binary
    rs = rs/255;

    if (is_test){
        plot_binary_img(" Stage 2 - After applying color threshold ", rs);
    }
    return rs;
}

Mat LaneDetector::combine_threshold(const Mat &s_binary,
                      const Mat &combined)
{
    Mat rs = (s_binary | combined);
    return rs ;
}

// =================== STAGE 3  ============================//
Mat LaneDetector::get_histogram(const Mat &binary_warped)
{
    int width = binary_warped.cols;
    int height = binary_warped.rows;
    Mat haft_bottom = binary_warped(Range(round(height/2), height), Range(0,width)); // row range, col range
    // plot_binary_img("haft_bottom", haft_bottom);
    Mat histogram = sum(haft_bottom,'x');

    if (!reduced)
    {
        plot_binary_img("Stage 3 - Get histogram of this region", haft_bottom);
        cv::imshow("Stage 3 - Histogram of image intensities", plot_histogram(histogram));
        // waitKey(1);
    }

    return histogram;

}

bool LaneDetector::slide_window(const Mat &binary_warped,
                              const Mat &histogram,
                              vector<Point2f> &center_windows,
                              vector<Point2f> &left_pts,
                            vector<Point2f> &right_pts)
{

    Mat hist = histogram.t(); // transpose

    int width = binary_warped.cols;
    int height = binary_warped.rows;

    Mat nonzero = findNonzero(binary_warped);

    // convert 2-D binary image into 2-D colored image (3channels)
    Mat out_img = dstack(binary_warped);

    // midpoint using multiple iterations T = (T1 + T2) / 2
    // int midpoint = round(hist.cols/2);
    int midpoint;
    if (turn_state != 2)// state 0,1
    {
        midpoint = find_midpoint(hist,5); // hist, min, max, eps
        if(midpoint == -1){
            midpoint = round(hist.cols/2);
        }
    }else{ // state 2 : turing
        midpoint = round(hist.cols/2);
    }
    // cout << "[INFO] midpoint = " << midpoint << "\n";

    // left boundary to the middle
    int leftx_current = arg_max(hist(Range(0, 1), Range(0,midpoint))).x; // row range, col range

    // right boundary to the middle
    int rightx_current = arg_max(hist(Range(0, 1), Range(midpoint,width))).x + midpoint; // row range, col range

    // #strwin
    for(int window = bwindow-1; window < ewindow; window++)
    {
        int win_y_low;
        int win_y_high = height - (window) * stridepix;
        if (window != nwindows - 1){
            win_y_low = win_y_high - window_height;
        }else{
            win_y_low = 0;
        }

        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;

        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;

        if (!reduced)
        {
            if (left_flag){
                cv::rectangle(out_img, Point(win_xleft_low,win_y_low),
                Point(win_xleft_high,win_y_high), Scalar(0,255,0), 2); // img, firstpoint, secondpoint, color,
            }
            if (right_flag){
                cv::rectangle(out_img, Point(win_xright_low,win_y_low),
                Point(win_xright_high,win_y_high), Scalar(0,255,0), 2);
            }
        }

        vector<int> leftxs; // which is inside the boxes
        vector<int> leftys;
        vector<int> rightxs;
        vector<int> rightys;

        // maintain only points which are inside this box
        for (int i = 0; i < nonzero.cols ; i++)
        {
            Point p = nonzero.at<Point>(0,i);
            if (left_flag)
            {
                if (is_inside_box(win_y_low, win_y_high,win_xleft_low, win_xleft_high, p))
                {
                    leftxs.push_back(p.x);
                    leftys.push_back(p.y);
                }
            }
            if (right_flag)
            {
                if (is_inside_box(win_y_low, win_y_high,win_xright_low, win_xright_high, p))
                {
                    rightxs.push_back(p.x);
                    rightys.push_back(p.y);
                }
            }
        }

        int num_lefts = leftxs.size();
        int num_rights = rightxs.size();

        double left_x, left_y, right_x, right_y;
        if  (num_lefts > minpix)
        {
            left_x = cv::mean(leftxs)[0];
            left_y = cv::mean(leftys)[0];
            leftx_current = round(left_x);
        }else{
            left_x = leftx_current;
            left_y = (float)(win_y_low - win_y_high)/2 + win_y_high;
        }

        if (num_rights > minpix)
        {
            right_x = cv::mean(rightxs)[0];
            right_y = cv::mean(rightys)[0];
            rightx_current = round(right_x);
        }else{
            right_x = rightx_current;
            right_y = (float)(win_y_low - win_y_high)/2 + win_y_high;
        }

        double center_x, center_y;
        if ((num_lefts > minpix) && (num_rights > minpix)){
            center_x = (left_x + right_x)/2;
            center_y = (left_y + right_y)/2;
        } else
        {
            center_x = (rightx_current - leftx_current)/2 + leftx_current;
            center_y = (float)(win_y_low - win_y_high)/2 + win_y_high;
        }
        center_windows.push_back(Point2f(center_x,center_y));
        left_pts.push_back(Point2f(left_x,left_y));
        right_pts.push_back(Point2f(right_x,right_y));
    }

    if(!reduced){
        imshow("Sliding Window", out_img);
        video->write(out_img);
        // waitKey(1);
    }

    return true;
}

int LaneDetector::find_midpoint(const Mat &hist, float eps)
{
    int w = hist.cols;

    int x_min, x_max;

    // finding x_min
    int i;
    for(i = 0; i < w; i++){
        if ((int)hist.at<uchar>(0,i) != 0){
            break;
        }
    }
    if (i == w-1){
        cout << "[ERROR] find x_min \n";
        return -1;
    }else{
        x_min = i;
    }

    // finding x_max
    for(i = w-1; i >= 0; i--){
        if ((int)hist.at<uchar>(0,i) != 0){
            break;
        }
    }
    if(i==0){
        cout << "[ERROR] find x_max \n";
        return -1;
    }else{
        x_max = i;
    }

    if(x_min > x_max){
        cout << "[ERROR] x min > x max \n";
        return -1;
    }

    float T = x_min + (float)(x_max-x_min)/2;
    while(true)
    {
        float m1 = calc_mean(hist, x_min, floor(T));
        float m2 = calc_mean(hist, floor(T), x_max);
        if ((m1 == -1)||(m2 == -1)){
            return -1;
        }
        float T_new = (m1+m2)/2;
        if (abs(T-T_new) < eps){
            return round(T);
        }
        T = T_new;
    }
}

double LaneDetector::calc_mean(const Mat &hist, int x_min, int x_max)
{
    double count = 0;
    double s = 0;
    for(int i = x_min; i < x_max; i++){
        count += (int)hist.at<uchar>(0,i);
        s += i * (int)hist.at<uchar>(0,i);
    }
    if (count == 0){
      cout << "[DEBUG] count = 0 !!" << " min = " << x_min << " max = " << x_max << "\n";
      return -1;
    }else{
      return s/count;
    }
}

// ==========================STAGE 4 ===============================

double LaneDetector::calc_angle(vector<Point2f> &center_windows,
                                vector<Point2f> &left_pts,
                                vector<Point2f> &right_pts,
                                Mat &gray)
{
    int ind = ewindow-bwindow;

    // upper
    double x1, y1;
    // lower
    double x2, y2;
    double angle;

    // state 0 (normal) , state 1 (braking)
    if(turn_state != 2)
    {
        x1 = center_windows[ind].x;
        y1 = center_windows[ind].y;
        x2 = (double)w/2;
        y2 = (double)h;
        angle = -atan((x2-x1)/(y2-y1))/M_PI * 180;
    }else {
        // state 2 : turning
        if(mode == 0) // turn left
        {
            x1 = left_pts[ind].x;
            y1 = left_pts[ind].y;
            x2 = left_pts[0].x;
            y2 = left_pts[0].y;
        }
        else // turn right
        {
            // int ind = floor((sign_ewindow-sign_bwindow)/2);
            ind = 1;
            double rx = right_pts[ind].x;
            x1 = (2*rx - ((w-rx)/marg)*((w-rx)/marg)*distance)/2;
            // x1 = (right_pts[ind].x*2-distance)/2;
            y1 = right_pts[ind].y;
            x2 = (double)w/2;
            y2 = (double)h;
        }
        angle = -atan((x2-x1)/(y2-y1))/M_PI * 180;
        // countdown
        if((angle < RANGE_COUNTDOWN_ANGLE[1]) &&
           (angle > RANGE_COUNTDOWN_ANGLE[0]))
        {
            countdown--;
            if(countdown <= 0) {
              turn_state=0; // non-sign
              mode = 2; // non-sign
              left_flag = true;
              right_flag = true;

              nwindows = non_nwindows;
              stridepix = non_stridepix;
              bwindow = non_bwindow;
              ewindow = non_ewindow;
            }
        }
    }
    // y = a x + b
    // x1    1    y1
    // x2    1    y2
    // (x2-x1) a = y2-y1
    // a = (y2-y1)/(x2-x1)

    line(gray,Point2f(x1,y1),Point2f(x2,y2),Scalar(0,255,0),5);
    imshow("Direction", gray);

    Mat tmp;
    cvtColor(gray,tmp,COLOR_GRAY2BGR);
    video2->write(tmp);

    // adjust angle in range RANGE_ANGLE
    if(angle < RANGE_ANGLE[0]) {angle = RANGE_ANGLE[0];}
    if(angle > RANGE_ANGLE[1]) {angle = RANGE_ANGLE[1];}
    return angle;
}

double LaneDetector::calc_speed(vector<Point2f> &center_windows,
                vector<Point2f> &left_pts,
              vector<Point2f> &right_pts)
{
    // state 1: normal speed 60
    if(turn_state==0) return normal_speed;

    // state 2: braking , speed 0
    if(turn_state==1){
        // waiting for speed down
        if (cur_speed < turn_speed+2)// 2 is the acceptable error
        {
            // move the turn state 2
            turn_state = 2;
            countdown = MAX_COUNTDOWN;
            if(mode==0) right_flag = false;
            if(mode==1) left_flag = false;
            cout << "[INFO] Switch to state 2: turning \n";

            // int ind = floor((non_ewindow-non_bwindow)/2);
            int ind = 1;
            // int ind = ewindow-bwindow;
            distance = right_pts[ind].x-left_pts[ind].x;
            if(mode==1) marg = w - right_pts[ind].x;
            if(mode==0) marg = left_pts[ind].x;

            nwindows = sign_nwindows;
            stridepix = sign_stridepix;
            bwindow = sign_bwindow;
            ewindow = sign_ewindow;
        }
        return 0.0;
    }

    // state 3: turing, speed 40
    if(turn_state==2) return turn_speed;

    return normal_speed;
}

// =================== HELPER FUNCTIONS ============================//
double LaneDetector::find_max(Mat img)
{
      double minVal;
      double maxVal;
      Point minLoc;
      Point maxLoc;
      cv::minMaxLoc(img, &minVal, &maxVal);
      return maxVal;
}

double LaneDetector::find_min(Mat img)
{
      double minVal;
      double maxVal;
      Point minLoc;
      Point maxLoc;
      cv::minMaxLoc(img, &minVal, &maxVal);
      return minVal;
}

void LaneDetector::show_min_max(string name , Mat img)
{
    std::cout << "  [INFO] Mat " << name << ", min = " << find_min(img) << ", max = " << find_max(img) << "\n";
}

Point LaneDetector::arg_max(Mat img)
{
      double minVal;
      double maxVal;
      Point minLoc;
      Point maxLoc;
      cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
      return maxLoc;
}

Mat LaneDetector::sum(Mat mat, char axis)
{
    int width = mat.cols;
    int height = mat.rows;
    Mat result ;
    Mat tmp;
    mat.convertTo(tmp, CV_64F);
    if (axis == 'x')
    {
        result = Mat(width,1,CV_8UC1);
        for (int x = 0 ; x < width; x++)
        {
            double s = 0;
            for (int y = 0; y < height; y++)
            {
                s += tmp.at<double>(y,x);
            }
            result.at<int>(x,0) = round(s);
        }
    }
    else
    {
        result = Mat(height,1,CV_8UC1);
        for (int y = 0 ; y < height; y++)
        {
            double s = 0;
            for (int x = 0; x < width; x++)
            {
                s += mat.at<double>(y,x);
            }
            result.at<int>(y,0) = round(s);
        }
    }

    return result ;
}

Mat LaneDetector::plot_histogram(Mat hist)
{

    hist.convertTo(hist, CV_64F, 1.0);

    // Establish the number of bins
    int histSize = hist.rows;

    // show_img_description("hist", hist);

    // Set the ranges
    float range[] = {0,256};
    const float* histRange = {range};

    // Draw the histograms for B, G and R
    int hist_w = histSize;
    int hist_h = 500;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    // show_img_description("hist",hist);

    // Draw
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<double>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(hist.at<double>(i)) ),
                         Scalar( 255, 0, 0), 2, 8, 0  );
    }

    // show_img_description("hist image", histImage);

    return histImage;
}

Mat LaneDetector::dstack(const Mat &img)
{
    int height = img.rows;
    int width = img.cols;

    Mat tmp;
    img.convertTo(tmp, CV_64F);
    Mat dstack_img(height, width, CV_8UC3, Vec3b(0,0,0));

    for (int y = 0 ; y < height; y++)
    {
        for(int x = 0 ; x < width; x++ )
        {
            int val = round(tmp.at<double>(y,x)) * 255;
            Vec3b& color = dstack_img.at<Vec3b>(y,x);
            color[0] = val ;
            color[1] = val ;
            color[2] = val ;
            // dstack_img.at<Vec3b>(Point(x,y)) = color;
        }
    }

    // show_img_description("TEST binary img", img);
    // show_img_description("TEST dstack ", dstack_iqmg);


    // Mat dstack_img;
    return dstack_img;

}

Mat LaneDetector::findNonzero(Mat img)
{
    int height = img.rows;
    int width = img.cols;

    img.convertTo(img, CV_64F);
    vector<Point> nonzero_pts ;
    for (int y = 0 ; y < height; y++)
    {
        for(int x = 0 ; x < width; x++ )
        {
            if (img.at<double>(y,x) > 0){
              nonzero_pts.push_back(Point(x,y));
            }
        }
    }

    Mat rs(1,nonzero_pts.size(), CV_64F);
    for (int x = 0 ; x < rs.cols; x++)
    {
        rs.at<Point>(0,x) = nonzero_pts[x];
    }

    return rs;
}

void LaneDetector::show_mat_type(string name, Mat &img) {
    string r;

    int type = img.type();

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    std::cout << " [INFO] Matrix " << name << " - " << r << "\n";
}

string LaneDetector::get_mat_type(Mat &img)
{
    string r;

    int type = img.type();

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

bool LaneDetector::is_inside_box(int win_y_low, int win_y_high,
                               int win_x_low, int win_x_high,
                               Point p)
{
    if (p.y >= win_y_low && p.y < win_y_high &&
        p.x >= win_x_low && p.x < win_x_high)
    {
        return true;
    }
    else return false;
}
//
// vector<double> LaneDetector::polyfit(vector<int> vecX, vector<int> vecY, int nDegree)
// {
//     // X and Y must be the same size
//
//
//     if(nDegree == 1)
//     {
//         vector<Point> pts ;
//         int sz = vecX.size();
//         for(int i = 0; i < sz; i++)
//         {
//           pts.push_back(Point(vecY[i], vecX[i]));
//         }
//
//         cv::Vec4f linear_line; // (-n, m) normalized vector collinear to the line , (x0,y0)
//
//         // cv::fitLine(pts, linear_line, CV_DIST_L2, 1, 0.001, 0.001);
//         // cv::fitLine(pts, linear_line, DIST_L2, 1, 0.001, 0.001);
//
//         // mx + ny + c = 0
//         double m = linear_line[1];
//         double n = -linear_line[0];
//         double x = linear_line[2];
//         double y = linear_line[3];
//         double c = - m * x - n * y;
//
//         vector<double> rs;
//         rs.push_back(-c/m);
//         rs.push_back(-n/m);
//
//         return rs;
//
//     }
//
//
//     int N = vecX.size();
//     double x[N],y[N];
//
//     for(int i = 0 ; i < N ; i++)
//     {
//         x[i] = (double)vecX[i];
//         y[i] = (double)vecY[i];
//     }
//
//     int n = nDegree;                         // n is the degree of Polynomial
//     double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
//     for (int i=0;i<2*n+1;i++)
//     {
//         X[i]=0;
//         for (int j=0;j<N;j++)
//             X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
//     }
//     double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
//     for (int i=0;i<=n;i++)
//         for (int j=0;j<=n;j++)
//             B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
//     double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
//     for (int i=0;i<n+1;i++)
//     {
//         Y[i]=0;
//         for (int j=0;j<N;j++)
//         Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
//     }
//     for (int i=0;i<=n;i++)
//         B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
//     n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
//     // if(is_test){
//     //   cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";
//     //   for (int i=0;i<n;i++)            //print the Normal-augmented matrix
//     //   {
//     //       for (int j=0;j<=n;j++)
//     //           cout<<B[i][j]<<setw(16);
//     //       cout<<"\n";
//     //   }
//     // }
//
//     for (int i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
//         for (int k=i+1;k<n;k++)
//             if (B[i][i]<B[k][i])
//                 for (int j=0;j<=n;j++)
//                 {
//                     double temp=B[i][j];
//                     B[i][j]=B[k][j];
//                     B[k][j]=temp;
//                 }
//
//     for (int i=0;i<n-1;i++)            //loop to perform the gauss elimination
//         for (int k=i+1;k<n;k++)
//             {
//                 double t=B[k][i]/B[i][i];
//                 for (int j=0;j<=n;j++)
//                     B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
//             }
//     for (int i=n-1;i>=0;i--)                //back-substitution
//     {                        //x is an array whose values correspond to the values of x,y,z..
//         a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
//         for (int j=0;j<n;j++)
//             if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
//                 a[i]=a[i]-B[i][j]*a[j];
//         a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
//     }
//
//     vector<double> coefs;
//     for (int i=0;i<n;i++){
//         coefs.push_back(a[i]);
//     }
//
//     // if(is_test){
//     //     cout<<"\n [INFO]\n";
//     //     for (int i=0;i<n;i++)
//     //         cout<<" + ("<<a[i]<<")"<<"y^"<<i;
//     //     cout<<"\n";
//     // }
//
//     return coefs;
//
// }

Mat LaneDetector::drawPolylines(Mat &img, vector<Point> pts)
{
    int height = img.rows;
    int sz = pts.size();
    Point ps[1][sz];
    for(int i = 0; i < sz; i++)
    {
        ps[0][i] = pts[i];
    }
    const Point* ppt[1] = {ps[0]};

    Mat drew_img(img);
    int numPts[] = {sz};
    fillPoly(drew_img, ppt, numPts, 1, Scalar(0,255,0), 8); // imgs, pts, numPts, ncontours, color,
    return drew_img;
}

vector<Point> LaneDetector::polyval(Mat &img, vector<double> coefs)
{

    int sz = coefs.size();

    int height = img.rows;
    vector<Point> rs;
    for (int i = 0 ; i < height; i++)
    {
        double y = double(i);
        double x ;
        if (sz == 2)
        {
            x = coefs[0] + coefs[1] * y ;
            rs.push_back(Point(round(x),round(y)));
        } else {


            // c + b y + a * y^2
            double a = coefs[2];
            double b = coefs[1];
            double c = coefs[0];
            double h = img.rows;
            x = (2*a*h + b)*y - a*h*h + c ;
            rs.push_back(Point(round(x),round(y)));

            // x = coefs[0] + coefs[1] * y + coefs[2] * y * y ;
            // rs.push_back(Point(round(x),round(y)));
        }


    }
    return rs;
}

vector<Point> LaneDetector::polyval(Mat &img, vector<double> coefs, bool is_pol)
{

    int sz = coefs.size();

    int height = img.rows;
    vector<Point> rs;
    for (int i = 0 ; i < height; i++)
    {
        double y = double(i);
        double x ;
        x = coefs[0] + coefs[1] * y + coefs[2] * y * y ;
        rs.push_back(Point(round(x),round(y)));
    }
    return rs;
}

void LaneDetector::show_histogram_normal(const Mat &img)
{
    vector<Mat> channels;

    // Separate the image in 3 places (for ex B,G,R)
    cv::split(img, channels);

    // Establish the number of bins
    int histSize = 256;

    // Set the ranges
    float range[] = {0,256};
    const float* histRange = {range};

    bool uniform = true;
    bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    /// Compute the histograms:
    calcHist( &channels[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &channels[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &channels[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    // Draw the histograms for B, G and R
    int hist_w = 512;
    int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    // Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                         Scalar( 255, 0, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                         Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                         Scalar( 0, 0, 255), 2, 8, 0  );
    }

    /// Display
    imshow("Histogram of 3 channels (b,g,r)", histImage );
    waitKey();

}

void LaneDetector::plot_binary_img(string name, const Mat &img)
{
    Mat plt_img(img);
    plt_img.convertTo(plt_img, CV_64F);
    cv::imshow(name, plt_img);
    // waitKey(1);
}

void LaneDetector::plot_binary_img(string name, const Mat &img, int wait_time)
{
    Mat plt_img(img);
    plt_img.convertTo(plt_img, CV_64F);
    cv::imshow(name, plt_img);
    waitKey(wait_time);
}

void LaneDetector::show_img_description(string name, const Mat &img)
{
    Mat tmp(img);
    cout <<"\n" << img << "\n";
    cout << " [INFO] width = " << img.cols << ", height = "<< img.rows << "\n";
    show_mat_type(name, tmp);
    show_min_max(name, tmp);
    cout << "\n" ;

    if(!reduced){
        if ((get_mat_type(tmp) == "8UC1") && ((find_max(tmp) == 1) || (find_min(tmp) == 0)))
        {
            plot_binary_img(" [Description] Binaric Image" , tmp);
        } else{
            imshow(" [Description] - Image", tmp);
            waitKey();
        }
    }
}

void LaneDetector::show_mat_per(string name, const Mat &img, char dir)
{
    cout << " Matrix " << name << ":\n" ;

    // if (dir == "col")
    // {
    //
    // }
    //
    // for(int x = 0;x < img.cols ; x++ )
    // {
    //     Range(round(height/2), height), Range(0,width)
    // }
}

void LaneDetector::videoProcess(string video_path)
{

    VideoCapture video(video_path);

    Mat img;
    Mat gray;
    cvtColor(img, gray,COLOR_RGB2GRAY);
    double angle=0, speed=-1;
    int iframe = 0;

    while (true)
    {
        video >> img;
        std::cout << " [INFO] Frame number " << iframe << "\n";
        if (img.empty()) {
            break;
        }
        imshow("View", img);
        waitKey(10);
        this->detect(img, gray,angle, speed);
        std::cout << " [INFO] angle = " << angle << ", speed =" << speed << "\n";

        iframe++;
    }
}
