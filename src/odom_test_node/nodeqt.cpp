#include "nodeqt.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <stdint-gcc.h>
#include <ros/time.h>

static NodeQt *globalThis = nullptr;

static void jointCallback(const sensor_msgs::JointState &msg)
{
    static uint64_t prevTime = 1507900970 * 10e9;
    uint64_t currentTime = msg.header.stamp.sec * 10e9 +
            msg.header.stamp.nsec;
    double dt = (currentTime - prevTime) / 10e9;
    prevTime = currentTime;
    globalThis->getPositions(msg.velocity[1], msg.position[0], dt);
}

static void laserCallback(const sensor_msgs::LaserScan &msg)
{
    globalThis->getLaser(msg.angle_min, msg.angle_increment,
                         msg.ranges);
}


static void odometryCallback(const nav_msgs::Odometry &msg)
{
    tf::Quaternion q(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    globalThis->setOdometry(msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            yaw);
}

NodeQt::NodeQt(int argc, char *argv[])
{
    globalThis = this;
    ros::init(argc, argv, "node_name");
    node_ = new ros::NodeHandle();
    velocityPublisher_ = node_->advertise<std_msgs::Float64>("/ur_hardware_driver/velocity_controller/command", 1000);
    steeringAnglePublisher_ = node_->advertise<std_msgs::Float64>("/ur_hardware_driver/steering_controller/command", 1000);
    jointSubscriber_ = node_->subscribe("/joint_states", 1000, jointCallback);
    laserSubscriber_ = node_->subscribe("/scan", 1000, laserCallback);
    odometrySubscriber_ = node_->subscribe("/rtabmap/odom", 1000,
                                           odometryCallback);
}

void NodeQt::getPositions(double velocity, double angle, double dt)
{
    velocity = velocity * 0.00385;
    if(angle <= 0)
    {
        angle = angle * (0.006135923151542565 / 1.95);
    }
    else
    {
        angle = angle * (0.006135923151542565 / 1.4);
    }
//    velocity = velocity * 0.00382;
//    angle = angle * (0.006135923151542565 / 2);

    Q_EMIT sendParams(velocity, angle, dt);
}

void NodeQt::getLaser(float angleMin, float angleIncrement,
                      const vector<float> &ranges)
{
    Q_EMIT sendLaser(angleMin, angleIncrement, ranges);
}


void NodeQt::setOdometry(double x, double y, double angle)
{
    Q_EMIT sendOdometry(y, x, -angle + M_PI / 2);
}

NodeQt::~NodeQt()
{

}

void NodeQt::run()
{
    ros::Rate loopRate(50);
//    float angleMin = -45 * M_PI / 180.;
//    float angleIncrement = 3 * M_PI / 180.;
//    vector<float> ranges;
//    for(int i = 0; i < 30; ++i)
//    {
//        ranges.push_back(2);
//    }
    while(ros::ok())
    {
        ros::spinOnce();
        //Q_EMIT sendParams(0.3, 0.5, 0.02);
        //Q_EMIT sendLaser(angleMin, angleIncrement, ranges);
        loopRate.sleep();
    }
}

void NodeQt::setVelocity(double velocity)
{
    std_msgs::Float64 message;
    message.data = velocity;
    velocityPublisher_.publish(message);
}

void NodeQt::setGetSteeringAngle(double angle)
{
    std_msgs::Float64 message;
    message.data = angle;
    steeringAnglePublisher_.publish(message);
}
