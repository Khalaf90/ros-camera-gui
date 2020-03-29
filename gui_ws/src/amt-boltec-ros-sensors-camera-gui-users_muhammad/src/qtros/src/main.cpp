/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date May 2019
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include <QApplication>
#include "../include/qtros/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    qtros::MainWindow w(argc,argv);
    w.setWindowTitle("MyAwesomeGUI");
    w.setWindowState(Qt::WindowMaximized);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
