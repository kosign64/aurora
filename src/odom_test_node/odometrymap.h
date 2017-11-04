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

    const double ROBOT_WIDTH;
    const double ROBOT_LENGTH;

Q_SIGNALS:

public Q_SLOTS:
    void setRobotPosition(double x, double y, double angle);
    void setLaser(float angleMin, float angleIncrement,
                  const vector<float> ranges);
};

#endif // ODOMETRYMAP_H
