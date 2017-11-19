#ifndef NODEQT_H
#define NODEQT_H

#include <ros/ros.h>
#include <vector>
#include <QThread>

using namespace std;

class NodeQt : public QThread
{
    Q_OBJECT
public:
    NodeQt(int argc, char *argv[]);
    void getPositions(double velocity, double angle, double dt);
    void getLaser(float angleMin, float angleIncrement,
                  const vector<float> &ranges);
    ~NodeQt();
    void setOdometry(double x, double y, double angle);

    void run();

private:
    ros::NodeHandle *node_;
    ros::Publisher velocityPublisher_;
    ros::Publisher steeringAnglePublisher_;
    ros::Subscriber jointSubscriber_;
    ros::Subscriber laserSubscriber_;
    ros::Subscriber odometrySubscriber_;

Q_SIGNALS:
    void sendParams(double velocity, double steeringAngle, double dt);
    void sendLaser(float angleMin, float angleIncrement,
                   const vector<float> ranges);
    void sendOdometry(double x, double y, double angle);

public Q_SLOTS:
    void setVelocity(double velocity);
    void setGetSteeringAngle(double angle);

};

#endif // NODEQT_H
