#ifndef GraphicalUserInterface_H
#define GraphicalUserInterface_H

#include "ui_GraphicalUserInterface.h"

#include <QMainWindow>
#include <opencv2/opencv.hpp>

#include <mutex>

namespace Ui {
    class MainWindow;
}

class GraphicalUserInterface : public QMainWindow, private Ui::GraphicalUserInterface
{
    Q_OBJECT
public:

    enum RecordSource{ UNKNOWN = 0,
        STEREO = 2,
        ULTRASOUND = 4,
        ARM = 8}; // Match logger class

    GraphicalUserInterface();

signals:
    /***
     * Record related
     * */
    void StartRecord(int);
    void StopRecord();

public slots:

    virtual void slotExit();

    void newStereoImage(QImage);

    void newUltrasoundImage(QImage);

    void setStatusMsg(QString msg);

    void setTimeCount(QString msg);

    void setArmPos(QString,QString,QString,QString);


private slots:

    void setInfo1(QString message);

    void setInfo2(QString message);

    void on_recordButton_released();

    void on_stopButton_released();


private:
    QPixmap m_green_icon;
    QPixmap m_red_icon;
    QPixmap m_hamlyn_logo;

};

#endif
