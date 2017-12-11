#ifndef NODEQT_H
#define NODEQT_H

#include <ros/ros.h>
#include <vector>
#include <QThread>
#include "common.h"

using namespace std;

class NodeQt : public QThread
{
    Q_OBJECT
public:
    NodeQt(int argc, char *argv[]);
    ~NodeQt();
    void setPositions(double velocity, double angle, double dt);
    void setLaser(float angleMin, float angleIncrement,
                  const vector<float> &ranges);
    void setMap(float xOrigin, float yOrigin, float angle,
                int32_t width, int32_t height, float resolution,
                const vector<int8_t> &mapData);
    void setOdometry(double x, double y, double angle);

    void run();

private:
    ros::NodeHandle *node_;
    ros::Publisher velocityPublisher_;
    ros::Publisher steeringAnglePublisher_;
    ros::Subscriber jointSubscriber_;
    ros::Subscriber laserSubscriber_;
    ros::Subscriber mapSubcriber_;
    ros::Subscriber odometrySubscriber_;

Q_SIGNALS:
    void sendParams(double velocity, double steeringAngle, double dt);
    void sendLaser(float angleMin, float angleIncrement,
                   const vector<float> ranges);
    void sendMap(float xOrigin, float yOrigin, float angle, int32_t width,
                 int32_t height, float resolution,
                 const vector<int8_t> map);
    void sendMapStruct(const Map &map);
    void sendOdometry(double x, double y, double angle);

public Q_SLOTS:
    void setVelocity(double velocity);
    void setSteeringAngle(double angle);

};

#endif // NODEQT_H
