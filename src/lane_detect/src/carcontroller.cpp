#include "carcontroller.h"

CarController::CarController()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("Team1_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("Team1_speed",10);
}

CarController::~CarController() {}

float CarController::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarController::driverCar(float _angle, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = _angle;
    speed.data = velocity;

    steer_publisher.publish(angle);
    usleep(1);
    speed_publisher.publish(speed);
    usleep(30);

    std::cout << " [INFO] Driving: angle = " << angle.data << ", speed = " << speed.data << std::endl;

}

// void CarController::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
// {
//     int i = left.size() - 11;
//     float error = preError;
//     // while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
//     //     i--;
//     //     if (i < 0) return;
//     // }
//     // if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
//     // {
//     //     error = errorAngle((left[i] + right[i]) / 2);
//     // }
//     // else if (left[i] != DetectLane::null)
//     // {
//     //     error = errorAngle(left[i] + Point(laneWidth / 2, 0));
//     // }
//     // else
//     // {
//     //     error = errorAngle(right[i] - Point(laneWidth / 2, 0));
//     // }
//
//     std_msgs::Float32 angle;
//     std_msgs::Float32 speed;
//
//     // angle.data = error;
//     //speed.data = velocity;
//
//     angle.data = -10.0;
//     speed.data = 200.0;
//
//     std::cout << " angle " << angle.data << " speed " << speed.data << std::endl;
//
//     steer_publisher.publish(angle);
//     speed_publisher.publish(speed);
// }

void CarController::auto_drive(int iframe)
{
    vector<double> lst = {-2};
    vector<int> len = {200};
}
