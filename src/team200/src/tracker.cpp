#include "tracker.h"

Tracker::Tracker()
{

}

Tracker::~Tracker()
{

}

void Tracker::init_tracker(const Mat &binary_warped,
                                Rect2d &up,
                                Rect2d &low)
{
    up_track_win = up;
    low_track_win = low;
    if(up.width == 0 || up.height == 0) cout << "[error] \n";
    if(low.width == 0 || low.height == 0) cout << "[error] \n";
}

void Tracker::tracking(const Mat &binary_warped,
                            Point2f &center_pts,
                            Mat &out_img)
{
    find_center_pts(binary_warped, center_pts,
                    up_track_win,
                    low_track_win,
                    out_img);
}

void Tracker::find_center_pts(const Mat &binary_warped,
                     Point2f &center_pts,
                     Rect2d &up,
                     Rect2d &low,
                     Mat &out_img)
{
    Mat nonzero = findNonzero(binary_warped);

    vector<int> upxs; // which is inside the boxes
    vector<int> upys;
    vector<int> lowxs;
    vector<int> lowys;

    // maintain only points which are inside this box
    for (int i = 0; i < nonzero.cols ; i++)
    {
        Point p = nonzero.at<Point>(0,i);
        if (is_inside_box(up.y,up.y+up.height,up.x,up.x+up.width,p))
        {
            upxs.push_back(p.x);
            upys.push_back(p.y);
        }

        if (is_inside_box(low.y,low.y+low.height,low.x,low.x+low.width, p))
        {
            lowxs.push_back(p.x);
            lowys.push_back(p.y);
        }

    }

    int num_ups = upxs.size();
    int num_lows = lowxs.size();

    double up_x, up_y, low_x, low_y;

    // upper
    if  (num_ups > minpix)
    {
        up_x = cv::mean(upxs)[0];
        up_y = cv::mean(upys)[0];
    }else{
        up_x = up.x + up.width/2;
        up_y = up.y + up.height/2;
    }

    // lower
    if (num_lows > minpix)
    {
        low_x = cv::mean(lowxs)[0];
        low_y = cv::mean(lowys)[0];
    }else{
        low_x = low.x + low.width/2;
        low_y = low.y + low.height/2;
    }

    center_pts = Point2f((up_x+low_x)/2,(up_y+low_y)/2);

    // update window
    up.x = up_x - up.width/2;
    up.y = up_y - up.height/2;
    low.x = low_x - low.width/2;
    low.y = low_y - low.height/2;

    rectangle(out_img,up,Scalar(0,255,0),4);
    rectangle(out_img,low,Scalar(0,255,0),4);
}


Mat Tracker::findNonzero(Mat img)
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

bool Tracker::is_inside_box(int win_y_low, int win_y_high,
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
