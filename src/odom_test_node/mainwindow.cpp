#include "mainwindow.h"
#include "nodeqt.h"
#include "odometrymap.h"
#include "astar.h"
#include "pathcontroller.h"
#include <QDoubleValidator>
#include <QKeyEvent>
#include <QLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>

MainWindow::MainWindow(int argc, char *argv[], QWidget *parent) :
    QMainWindow(parent),
    move_(false),
    xPosition_(0),
    yPosition_(0),
    anglePosition_(0),
    externalOdometry_(false),
    pathControl_(false)
{
    qRegisterMetaType< vector<float> >("vector<float>");
    mainWidget_ = new QWidget(this);
    mainLayout_ = new QGridLayout(mainWidget_);

    startStopButton_ = new QPushButton("Start robot", mainWidget_);
    angleSlider_ = new QSlider(Qt::Horizontal, mainWidget_);
    velocityEdit_ = new QLineEdit("120", mainWidget_);
    estimatedPositionLabel_ = new QLabel("X: 0м, Y: 0м, angle: 0", mainWidget_);
    resetOdometryButton_ = new QPushButton("Reset odometry", mainWidget_);
    selectOdometryButton_ = new QPushButton("External odometry", mainWidget_);
    selectControlButton_ = new QPushButton("Path control", mainWidget_);
    map_ = new OdometryMap(mainWidget_);

    QDoubleValidator *validator = new QDoubleValidator(-120.0, 120.0, 2, this);
    validator->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
    velocityEdit_->setValidator(validator);

    angleSlider_->setRange(-90, 90);
    angleSlider_->setValue(0);
    angleSlider_->setTickPosition(QSlider::TicksAbove);
    angleSlider_->setTickInterval(10);
    angleSlider_->setFixedWidth(500);

    mainLayout_->addWidget(startStopButton_, 0, 0, 1, 1, Qt::AlignCenter);
    mainLayout_->addWidget(velocityEdit_, 0, 1, 1, 1, Qt::AlignCenter);
    mainLayout_->addWidget(angleSlider_, 1, 0, 1, 2, Qt::AlignCenter);
    mainLayout_->addWidget(selectOdometryButton_,2, 0, 1, 1, Qt::AlignCenter);
    mainLayout_->addWidget(resetOdometryButton_, 2, 1, 1, 1, Qt::AlignCenter);
    mainLayout_->addWidget(selectControlButton_, 3, 0, 1, 2, Qt::AlignCenter);
    mainLayout_->addWidget(estimatedPositionLabel_, 4, 0, 1, 2, Qt::AlignCenter);
    mainLayout_->addWidget(map_, 5, 0, 2, 2, Qt::AlignCenter);

    mainWidget_->setLayout(mainLayout_);

    setCentralWidget(mainWidget_);

    rosNode_ = new NodeQt(argc, argv);

    connect(startStopButton_, &QPushButton::clicked,
            this, &MainWindow::onStartStopButtonClick);
    connect(angleSlider_, &QSlider::valueChanged,
            this, &MainWindow::onAngleSliderValueChange);
    connect(velocityEdit_, &QLineEdit::textChanged,
            this, &MainWindow::onVelocityEditValueChange);
    connect(resetOdometryButton_, &QPushButton::clicked,
            this, &MainWindow::onResetOdometryButtonClick);
    connect(selectOdometryButton_, &QPushButton::clicked, this,
            &MainWindow::onSelectOdometryButtonClick);
    connect(selectControlButton_, &QPushButton::clicked, this,
            &MainWindow::onSelectControlButtonClick);

    connect(this, &MainWindow::setVelocity, rosNode_, &NodeQt::setVelocity);
    connect(this, &MainWindow::setSteeringAngle,
            rosNode_, &NodeQt::setSteeringAngle);

    connect(rosNode_, &NodeQt::sendParams, this,
            &MainWindow::setParams);

    connect(this, &MainWindow::sendRobotPosition, map_,
            &OdometryMap::setRobotPosition);

    connect(rosNode_, &NodeQt::sendLaser, map_,
            &OdometryMap::setLaser);

    qRegisterMetaType<int32_t>("int32_t");
    qRegisterMetaType< vector<int8_t> >("vector<int8_t>");
    qRegisterMetaType<Map>("Map");
    connect(rosNode_, &NodeQt::sendMapStruct, map_,
            &OdometryMap::setMapStruct);

    astar_ = new AStar(this);

    qRegisterMetaType< vector<Point2D> >("vector<Point2D>");

    connect(rosNode_, &NodeQt::sendOdometry, astar_,
            &AStar::setOdometry);
    connect(rosNode_, &NodeQt::sendMapStruct, astar_,
            &AStar::setMap);
    connect(map_, &OdometryMap::pressedPoint, astar_,
            &AStar::setStopPoint);
    connect(astar_, &AStar::sendPath, map_,
            &OdometryMap::setPoints);

    pathController_ = new PathController(this);

    connect(rosNode_, &NodeQt::sendOdometry, pathController_,
            &PathController::setRobotPosition);
    connect(astar_, &AStar::sendPath, pathController_,
            &PathController::setPath);

    rosNode_->start();

    Q_EMIT setVelocity(0);
    Q_EMIT setSteeringAngle(0);
}

MainWindow::~MainWindow()
{
    Q_EMIT setVelocity(0);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Escape)
    {
        close();
    }
    else if(event->key() == Qt::Key_W)
    {
        if(!move_)
        {
            if(velocityEdit_->text().toDouble() < 0)
            {
                velocityEdit_->setText(
                            QString::number(-velocityEdit_->text().toDouble()));
            }
            onStartStopButtonClick();
        }
    }
    else if(event->key() == Qt::Key_A)
    {
        angleSlider_->setValue(-90);
    }
    else if(event->key() == Qt::Key_D)
    {
        angleSlider_->setValue(90);
    }
    else if(event->key() == Qt::Key_S)
    {
        if(!move_)
        {
            if(velocityEdit_->text().toDouble() > 0)
            {
                velocityEdit_->setText(
                            QString::number(-velocityEdit_->text().toDouble()));
            }
            onStartStopButtonClick();
        }
    }
    QWidget::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if((event->key() == Qt::Key_W) || (event->key() == Qt::Key_S))
    {
        if(move_)
        {
            onStartStopButtonClick();
        }
    }
    else if((event->key() == Qt::Key_A) || (event->key() == Qt::Key_D))
    {
        angleSlider_->setValue(0);
    }
}

void MainWindow::setParams(double velocity, double steeringAngle, double dt)
{
    static const double LENGTH = 0.68; // Measured!
    xPosition_ += dt * velocity * sin(anglePosition_);
    yPosition_ += dt * velocity * cos(anglePosition_);
    anglePosition_ += dt * (velocity / LENGTH) * tan(steeringAngle);
    Q_EMIT sendRobotPosition(xPosition_, yPosition_, anglePosition_);
    QString positionString = "X: " + QString::number(xPosition_, 'g', 3) +
            "м Y: " + QString::number(yPosition_, 'g', 3) + "м angle: " +
            QString::number(anglePosition_ * 180. / M_PI, 'g', 3);
    estimatedPositionLabel_->setText(positionString);
}

void MainWindow::onStartStopButtonClick()
{
    move_ = !move_;
    if(move_)
    {
        startStopButton_->setText("Stop robot");
        Q_EMIT setVelocity(velocityEdit_->text().toDouble());
    }
    else
    {
        startStopButton_->setText("Start robot");
        Q_EMIT setVelocity(0);
    }
}

void MainWindow::onAngleSliderValueChange()
{
    Q_EMIT setSteeringAngle(angleSlider_->value());
}

void MainWindow::onVelocityEditValueChange()
{
    if(move_)
    {
        Q_EMIT setVelocity(velocityEdit_->text().toDouble());
    }
}

void MainWindow::onResetOdometryButtonClick()
{
    xPosition_ = 0.;
    yPosition_ = 0.;
    anglePosition_ = 0.;
}

void MainWindow::onSelectOdometryButtonClick()
{
    if(externalOdometry_)
    {
        externalOdometry_ = false;
        selectOdometryButton_->setText("External odometry");
        disconnect(rosNode_, &NodeQt::sendOdometry, map_,
                   &OdometryMap::setRobotPosition);
        connect(this, &MainWindow::sendRobotPosition, map_,
                           &OdometryMap::setRobotPosition);
    }
    else
    {
        externalOdometry_ = true;
        selectOdometryButton_->setText("Internal odometry");
        disconnect(this, &MainWindow::sendRobotPosition, map_,
                   &OdometryMap::setRobotPosition);
        connect(rosNode_, &NodeQt::sendOdometry, map_,
                &OdometryMap::setRobotPosition);
    }
}

void MainWindow::onSelectControlButtonClick()
{
    if(pathControl_)
    {
        pathControl_ = false;
        selectControlButton_->setText("Path control");
        disconnect(pathController_, &PathController::sendVelocity,
                   rosNode_, &NodeQt::setVelocity);
        disconnect(pathController_, &PathController::sendSteeringAngle,
                   rosNode_, &NodeQt::setSteeringAngle);
        connect(this, &MainWindow::setVelocity, rosNode_,
                &NodeQt::setVelocity);
        connect(this, &MainWindow::setSteeringAngle, rosNode_,
                &NodeQt::setSteeringAngle);
    }
    else
    {
        pathControl_ = true;
        selectControlButton_->setText("Manual control");
        disconnect(this, &MainWindow::setVelocity, rosNode_,
                &NodeQt::setVelocity);
        disconnect(this, &MainWindow::setSteeringAngle, rosNode_,
                &NodeQt::setSteeringAngle);
        connect(pathController_, &PathController::sendVelocity,
                   rosNode_, &NodeQt::setVelocity);
        connect(pathController_, &PathController::sendSteeringAngle,
                   rosNode_, &NodeQt::setSteeringAngle);
    }
}

