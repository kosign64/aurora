#ifndef ODOMETRYMAP_H
#define ODOMETRYMAP_H

#include <QWidget>
#include <vector>

using namespace std;

class OdometryMap : public QWidget
{
    Q_OBJECT
public:
    explicit OdometryMap(QWidget *parent = nullptr);
    QSize sizeHint() const {return QSize(1200, 600);}

protected:
    void timerEvent(QTimerEvent *);
    void paintEvent(QPaintEvent *);

private:
    double xPosition_;
    double yPosition_;
    double anglePosition_;

    float laserAngleMin_;
    float laserAngleIncrement_;
    vector<float> laserRanges_;

    vector<int8_t> map_;
    float mapResolution_;
    int32_t mapWidth_;
    int32_t mapHeight_;
    float mapX_;
    float mapY_;
    float mapAngle_;
    const double ROBOT_WIDTH;
    const double ROBOT_LENGTH;

    void drawMap(QPainter &painter, const double scaleMeter);
Q_SIGNALS:

public Q_SLOTS:
    void setRobotPosition(double x, double y, double angle);
    void setLaser(float angleMin, float angleIncrement,
                  const vector<float> ranges);
    void setMap(float xOrigin, float yOrigin, float angle, int32_t width,
                int32_t height, float resolution,
                const vector<int8_t> map);
};

#endif // ODOMETRYMAP_H
