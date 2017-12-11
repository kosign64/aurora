#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <QObject>
#include <vector>
#include "common.h"

class PathController : public QObject
{
    Q_OBJECT
public:
    explicit PathController(QObject *parent = nullptr);
    ~PathController();
    void calcControl();

private:
    Point2D robotPosition_;
    double robotAngle_;
    std::vector<Point2D> path_;

Q_SIGNALS:
    void sendVelocity(double velocity);
    void sendSteeringAngle(double angle);

public Q_SLOTS:
    void setRobotPosition(double x, double y, double angle);
    void setPath(const std::vector<Point2D> &path) {path_ = path;}
};

#endif // PATHCONTROLLER_H
