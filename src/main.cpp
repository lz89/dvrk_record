#include <QApplication>
#include <QDebug>
#include <QTimer>
#include <QThread>

#include "qnode.h"
#include "GraphicalUserInterface.h"
#include "logger.h"

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(dvrk::GraphicalUserInterface* gui
                     , dvrk::Logger* logger)
{

    // Connect button signal to appropriate slot
    QObject::connect(gui, SIGNAL(StartRecord()), logger, SLOT(start_record()));
    QObject::connect(gui, SIGNAL(StopRecord()), logger, SLOT(stop_record()));

    QObject::connect(logger,  SIGNAL(newStereoImage(QImage)), gui, SLOT(newStereoImage(QImage)));
    QObject::connect(logger,  SIGNAL(newUltrasoundImage(QImage)), gui, SLOT(newUltrasoundImage(QImage)));
    QObject::connect(logger,  SIGNAL(setStatusMsg(QString)), gui, SLOT(setStatusMsg(QString)));
    QObject::connect(logger,  SIGNAL(setInfo1(QString)), gui, SLOT(setInfo1(QString)));
    QObject::connect(logger,  SIGNAL(setInfo2(QString)), gui, SLOT(setInfo2(QString)));
    QObject::connect(logger,  SIGNAL(setTimeCount(QString)), gui, SLOT(setTimeCount(QString)));

}

int main(int argc, char *argv[])
{
    QNode qtRos (argc, argv, "dvrk_record");

    QApplication app(argc, argv);
    QIcon appIcon;
    appIcon.addFile(":/icons/logo.png");
    app.setWindowIcon(appIcon);

    /**
     * @brief logger
     */
    dvrk::Logger logger;



    dvrk::GraphicalUserInterface *gui = new dvrk::GraphicalUserInterface();
    gui->show();
    gui_connections(gui, &logger);

    QThread t;
    QTimer timer;
    QObject::connect(&timer, SIGNAL(timeout()), &logger, SLOT(process_loop()));
    timer.start(10);
    logger.moveToThread(&t);

    //If one thread receives a exit signal from the user, signal the other thread to quit too
    QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
    QObject::connect(&app, SIGNAL(aboutToQuit()), &t, SLOT(quit()));
    QObject::connect(&t, SIGNAL(finished()), &t, SLOT(deleteLater()));
    QObject::connect(&qtRos, SIGNAL(rosShutdown()), &app, SLOT(quit()));

    t.start();
    qtRos.start();// Run main loop.
    return app.exec();
}
