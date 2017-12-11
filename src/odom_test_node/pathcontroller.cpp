#include "pathcontroller.h"
#include <ros/ros.h>

static double radToGrad(double rad)
{
    return rad * 180. / M_PI;
}

PathController::PathController(QObject *parent) : QObject(parent),
    robotPosition_{-100, -100}
{

}

PathController::~PathController()
{
    Q_EMIT sendVelocity(0);
}

void PathController::calcControl()
{
    if((robotPosition_.x == -100 && robotPosition_.y == -100) ||
            (path_.size() < 2)) return;
    const auto &currentPoint = path_[0];
    const auto &nextPoint = path_[1];
    double desiredAngle = atan2(currentPoint.y - nextPoint.y,
                         -currentPoint.x + nextPoint.x);
    ROS_INFO_STREAM("DA: " << radToGrad(desiredAngle) <<
                    " RA: " << radToGrad(robotAngle_) <<
                    " MRA: " << radToGrad(robotAngle_ - M_PI_2));
    double angle = desiredAngle - (robotAngle_ - M_PI_2);
    if(angle > M_PI) angle -= 2 * M_PI;
    if(angle < -M_PI) angle += 2 * M_PI;
    angle *= 90. / (25 * M_PI / 180.); // 25 degrees = 90 points
    if(angle > 90) angle = 90;
    else if(angle < -90) angle = -90;
    ROS_INFO_STREAM("Control angle" << angle);
    const auto &endPoint = path_.back();
    if(sqrt(pow(robotPosition_.x - endPoint.x, 2) +
            pow(robotPosition_.y - endPoint.y, 2)) < 1)
    {
        Q_EMIT sendVelocity(0);
    }
    else
    {
        Q_EMIT sendVelocity(40);
    }
    Q_EMIT sendSteeringAngle(angle);
}

void PathController::setRobotPosition(double x, double y, double angle)
{
    robotPosition_ = Point2D{static_cast<float>(x),
                             static_cast<float>(y)};
    robotAngle_ = angle;
    calcControl();
}
