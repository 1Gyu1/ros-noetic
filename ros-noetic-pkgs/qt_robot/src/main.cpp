/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/


#include <QtGui>
#include <QApplication>
#include "mainwindow.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    return app.exec();
}
