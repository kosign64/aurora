#include <QCoreApplication>
#include "nodeqt.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    NodeQt node(argc, argv);
    node.start();
    a.connect(&node, &NodeQt::quit,
              &a, &QCoreApplication::quit);

    return a.exec();
}
