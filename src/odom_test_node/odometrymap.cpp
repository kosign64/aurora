#include "odometrymap.h"
#include <ros/ros.h>
#include <QPainter>

OdometryMap::OdometryMap(QWidget *parent) : QWidget(parent),
    xPosition_(0),
    yPosition_(0),
    anglePosition_(0),
    // Measured!
    ROBOT_LENGTH(1),
    ROBOT_WIDTH(0.65)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    startTimer(1000 / 50);
}

void OdometryMap::timerEvent(QTimerEvent *)
{
    update();
}

void OdometryMap::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const double scaleMeter = width() / 26.;
    QPen pen(Qt::white);
    QBrush brush(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.setBrush(brush);
    painter.drawRect(0, 0, width(), height());
    painter.setPen(pen);
    painter.translate(width() / 2., height() / 2.);
    for(double x = (-width() / 2.); x < (width() / 2.); x += scaleMeter)
    {
        painter.drawLine(x, -height() / 2., x, height() / 2.);
    }
    for(double y = 0; y < (height() / 2.); y += scaleMeter)
    {
        painter.drawLine((-width() / 2.), y, (width() / 2.), y);
    }
    for(double y = 0; y > (-height() / 2.); y -= scaleMeter)
    {
        painter.drawLine((-width() / 2.), y, (width() / 2.), y);
    }
    // X - forward moving
    painter.translate(yPosition_  * scaleMeter, -xPosition_  * scaleMeter);
    painter.rotate(anglePosition_ * 180 / M_PI);
    brush.setColor(Qt::gray);
    pen.setColor(Qt::green);
    pen.setWidth(3);
    painter.setBrush(brush);
    painter.setPen(pen);
    const double robotWidth = ROBOT_WIDTH * scaleMeter;
    const double robotLength = ROBOT_LENGTH * scaleMeter;
    painter.drawRect(QRectF(-robotWidth / 2, -robotLength / 2,
                     robotWidth, robotLength));
    brush.setColor(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.setBrush(brush);
    painter.drawRect(-robotWidth / 4, -robotLength / 2,
                     robotWidth / 2, robotWidth / 2);
    if(laserRanges_.empty()) return;
    painter.translate(0, -robotLength / 2.);
    brush.setColor(Qt::green);
    painter.setBrush(brush);
    painter.drawEllipse(QPointF(0, 0), 0.2 * scaleMeter,
                        0.2 * scaleMeter);
    painter.rotate(laserAngleMin_ * 180. / M_PI);
    const double angleStep = laserAngleIncrement_ * 180. / M_PI;
    brush.setColor(Qt::red);
    painter.setBrush(brush);
    for(const auto &range : laserRanges_)
    {
        painter.drawEllipse(QPointF(0, -range * scaleMeter),
                            0.05 * scaleMeter, 0.05 * scaleMeter);
        painter.rotate(angleStep);
    }
}

void OdometryMap::setRobotPosition(double x, double y, double angle)
{
    xPosition_ = x;
    yPosition_ = y;
    anglePosition_ = angle;
}

void OdometryMap::setLaser(float angleMin, float angleIncrement,
                           const vector<float> ranges)
{
    laserAngleMin_ = angleMin;
    laserAngleIncrement_ = angleIncrement;
    laserRanges_ = ranges;
}

