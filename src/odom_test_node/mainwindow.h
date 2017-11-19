#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class NodeQt;
class OdometryMap;
class QGridLayout;
class QPushButton;
class QSlider;
class QLineEdit;
class QLabel;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(int argc, char *argv[], QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

private:
    // GUI
    QWidget *mainWidget_;
    QGridLayout *mainLayout_;
    // Widgets
    QPushButton *startStopButton_;
    QSlider *angleSlider_;
    QPushButton *selectOdometryButton_;
    QPushButton *resetOdometryButton_;
    QLineEdit *velocityEdit_;
    QLabel *estimatedPositionLabel_;
    OdometryMap *map_;

    bool move_;

    double xPosition_;
    double yPosition_;
    double anglePosition_;

    NodeQt *rosNode_;

    bool externalOdometry_;

Q_SIGNALS:
    void setVelocity(double velocity);
    void setSteeringAngle(double angle);
    void sendRobotPosition(double x, double y, double angle);

private Q_SLOTS:
    void setParams(double velocity, double steeringAngle, double dt);

    void onStartStopButtonClick();
    void onAngleSliderValueChange();
    void onVelocityEditValueChange();
    void onResetOdometryButtonClick();
    void onSelectOdometryButtonClick();
};

#endif // MAINWINDOW_H
