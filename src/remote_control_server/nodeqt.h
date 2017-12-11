#ifndef NODEQT_H
#define NODEQT_H

#include <ros/ros.h>
#include <vector>
#include <QThread>

class QTcpServer;
class QTcpSocket;

using namespace std;

class NodeQt : public QThread
{
    Q_OBJECT
public:
    NodeQt(int argc, char *argv[]);
    ~NodeQt();

    void run();

private:
    ros::NodeHandle *node_;
    ros::Publisher velocityPublisher_;
    ros::Publisher steeringAnglePublisher_;

    QTcpServer *server_;
    QTcpSocket *socket_;
    bool connected_;

Q_SIGNALS:
    void quit();

public Q_SLOTS:
    void onConnected();
    void onReadyRead();
    void onDisconnected();
};

#endif // NODEQT_H
