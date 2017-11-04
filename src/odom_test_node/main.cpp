#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.show();
    app.connect(&app, &QApplication::lastWindowClosed,
                &app, &QApplication::quit);

    return app.exec();
}
