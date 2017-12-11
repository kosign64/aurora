#include "nodeqt.h"
#include <std_msgs/Float64.h>
#include <stdint-gcc.h>
#include <ros/time.h>
#include <QTcpServer>
#include <QTcpSocket>

NodeQt::NodeQt(int argc, char *argv[]) :
    connected_(false)
{
    ros::init(argc, argv, "aurora_remote_control");
    node_ = new ros::NodeHandle();
    velocityPublisher_ = node_->advertise<std_msgs::Float64>("/ur_hardware_driver/velocity_controller/command", 1000);
    steeringAnglePublisher_ = node_->advertise<std_msgs::Float64>("/ur_hardware_driver/steering_controller/command", 1000);

    server_ = new QTcpServer(this);
    if(!server_->listen(QHostAddress::Any, 4444))
    {
        ROS_FATAL_STREAM("Error: Can't open port!");
        exit(-1);
    }
    connect(server_, &QTcpServer::newConnection, this,
            &NodeQt::onConnected);
}

NodeQt::~NodeQt()
{
    server_->close();
    std_msgs::Float64 message;
    message.data = 0;
    velocityPublisher_.publish(message);
}

void NodeQt::run()
{
    ros::Rate loopRate(10);
    while(ros::ok())
    {
        loopRate.sleep();
    }
    Q_EMIT quit();
}

void NodeQt::onConnected()
{
    if(!connected_)
    {
        socket_ = server_->nextPendingConnection();
        connect(socket_, &QTcpSocket::readyRead, this,
                &NodeQt::onReadyRead);
        connect(socket_, &QTcpSocket::disconnected, this,
                &NodeQt::onDisconnected);
        connected_ = true;
    }
    else
    {
        QTcpSocket *socket = server_->nextPendingConnection();
        socket->disconnectFromHost();
    }
}

void NodeQt::onReadyRead()
{
    if(socket_->bytesAvailable() >= 2)
    {
        int8_t data[2];
        socket_->read((char*)&data, 2);
        ROS_INFO_STREAM("Angle: " << int(data[1]) <<
                        "Speed: " << int(data[0]));
        std_msgs::Float64 message;
        message.data = data[0];
        velocityPublisher_.publish(message);
        message.data = data[1];
        steeringAnglePublisher_.publish(message);
    }
}

void NodeQt::onDisconnected()
{
    disconnect(socket_, &QTcpSocket::readyRead, this,
               &NodeQt::onReadyRead);
    disconnect(socket_, &QTcpSocket::disconnected, this,
               &NodeQt::onDisconnected);
    socket_->deleteLater();
    connected_ = false;
}
