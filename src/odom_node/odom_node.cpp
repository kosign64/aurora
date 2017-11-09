#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

using namespace ros;

double x = 0.;
double y = 0.;
double angle = 0.;
double vx = 0.;
double vy = 0.;
double vangle = 0.;

static void jointCallback(const sensor_msgs::JointState &msg)
{
    static const double LENGTH = 0.68;
    static uint64_t prevTime = 1507900970 * 10e9;
    static double prevX = 0.;
    static double prevY = 0.;
    static double prevAngle = 0.;
    uint64_t currentTime = msg.header.stamp.sec * 10e9 + msg.header.stamp.nsec;
    double dt = (currentTime - prevTime) / 10e9;

    double velocity = msg.velocity[1] * 0.00385;
    double steeringAngle;
    if(msg.position[0] <= 0)
    {
        steeringAngle = -msg.position[0] * (0.006135923151542565 / 1.95);
    }
    else
    {
        steeringAngle = -msg.position[0] * (0.006135923151542565 / 1.4);
    }

    if(dt > 0)
    {
        x += dt * velocity * cos(angle);
        y += dt * velocity * sin(angle);
        angle += dt * (velocity / LENGTH) * tan(steeringAngle);

        vx = (x - prevX) / dt;
        vy = (y - prevY) / dt;
        vangle = (angle - prevAngle) / dt;

        ROS_INFO_STREAM("dt " << currentTime - prevTime << " vx " << vx << " vy " << vy << " " << velocity);

        prevX = x;
        prevY = y;
        prevAngle = angle;
        prevTime = currentTime;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aurora_odom");

    NodeHandle nodeHandle;
    Publisher odomPub = nodeHandle.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odomBroadcaster;
    Subscriber jointSubscriber = nodeHandle.subscribe("/joint_states", 1000,
                                                      jointCallback);

    Time lastTime = Time::now();
    Time currentTime = Time::now();

    Rate rate(50);

    while(nodeHandle.ok())
    {
        spinOnce();
        currentTime = Time::now();

        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(angle);

        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = currentTime;
        odomTrans.header.frame_id = "odom";
        odomTrans.child_frame_id = "base_link";

        odomTrans.transform.translation.x = x;
        odomTrans.transform.translation.y = y;
        odomTrans.transform.translation.z = 0.;
        odomTrans.transform.rotation = odomQuat;

        odomBroadcaster.sendTransform(odomTrans);

        nav_msgs::Odometry odom;
        odom.header.stamp = currentTime;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odomQuat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vangle;

        odomPub.publish(odom);

        lastTime = currentTime;
        rate.sleep();
    }

    return 0;
}
